#include "wrapper.hpp"

/* Include Begin */
#include <iostream>
#include <string>
#include <vector>
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "DUALSHOCK2.hpp"
#include "TC74HC595.hpp"
#include "mcp3208.hpp"

#include "Maerobo2022.hpp"
/* Include End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
struct Timer{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
};

struct Gpio{
	GPIO_TypeDef * port;
	uint16_t pin;
};
/* Struct End */

/* Variable Begin */
std::array<uint8_t, 21> receive_data;
std::array<int16_t, 4> stick_data;
uint8_t debug_receive_data[21];
int16_t debug_stick_data[4];
uint16_t sr_data;
const std::vector<Timer> md_pwm = {
		{&htim4,  TIM_CHANNEL_4},
		{&htim4,  TIM_CHANNEL_3},
		{&htim15, TIM_CHANNEL_1},
		{&htim15, TIM_CHANNEL_2},
		{&htim12, TIM_CHANNEL_1},
		{&htim12, TIM_CHANNEL_2},
		{&htim13, TIM_CHANNEL_1},
		{&htim14, TIM_CHANNEL_1}
};
std::array<int16_t, 8> md_compare;
int16_t debug_md_compare[8];
std::array<uint16_t, 8> adc_value_array;
uint16_t debug_adc_value_array[8];

const std::vector<Gpio> sensor = {
		{GPIOC, GPIO_PIN_10}, // 1
		{GPIOC, GPIO_PIN_11}, // 2
		{GPIOA, GPIO_PIN_0},  // 3
		{GPIOA, GPIO_PIN_1},  // 4
		{GPIOA, GPIO_PIN_11}, // 5
		{GPIOA, GPIO_PIN_8},  // 6
		{GPIOC, GPIO_PIN_7},  // 7
		{GPIOC, GPIO_PIN_6},  // 8
		{GPIOA, GPIO_PIN_15}, // 9
		{GPIOF, GPIO_PIN_7},  // 10
		{GPIOF, GPIO_PIN_6},  // 11
		{GPIOA, GPIO_PIN_12}, // 12
		{GPIOA, GPIO_PIN_10}, // 13
		{GPIOA, GPIO_PIN_9},  // 14
		{GPIOC, GPIO_PIN_9},  // 15
		{GPIOC, GPIO_PIN_8},  // 16
};
const std::vector<Gpio> led = {
		{LED_01_GPIO_Port, LED_01_Pin},
		{LED_02_GPIO_Port, LED_02_Pin},
		{LED_03_GPIO_Port, LED_03_Pin},
		{LED_04_GPIO_Port, LED_04_Pin},
		{LED_05_GPIO_Port, LED_05_Pin},
		{LED_06_GPIO_Port, LED_06_Pin},
		{LED_07_GPIO_Port, LED_07_Pin},
		{LED_08_GPIO_Port, LED_08_Pin},
};
mb_22::Maerobo_State state;
/* Variable End */

/* Class Constructor Begin */
DUALSHOCK2 dualshock2(hspi1, SPI1_SS_GPIO_Port, SPI1_SS_Pin, 0xF);
tc74hc595::TC74HC595 tc74hc595_writer({SR_SI_GPIO_Port, SR_SI_Pin}, {SR_SCK_GPIO_Port, SR_SCK_Pin}, {SR_RCK_GPIO_Port, SR_RCK_Pin});
mcp3208::MCP3208 mcp3208_reader(hspi2,SPI2_SS_GPIO_Port,SPI2_SS_Pin);

mb_22::Maerobo_2022 maerobo_2022;
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
	dualshock2.init();
	dualshock2.reset_stick();
	tc74hc595_writer.init();
	for (uint8_t i=0; i < 8; i++) {
		HAL_TIM_PWM_Start(md_pwm[i].htim, md_pwm[i].channel);
	}
	mcp3208_reader.init();

	for (int i = 0; i < 8; ++i) {
		HAL_GPIO_WritePin(led[i].port, led[i].pin, GPIO_PIN_SET);
	}
	HAL_Delay(1000);
	for (int i = 0; i < 8; ++i) {
		HAL_GPIO_WritePin(led[i].port, led[i].pin, GPIO_PIN_RESET);
	}

	HAL_TIM_Base_Start_IT(&htim18); // メイン処理を受け持つタイマー割込み
}

void loop(void){
	// コントローラ読み取り
	dualshock2.update(0x30);
	receive_data = dualshock2.get_data_exex();
	std::copy(receive_data.begin(), receive_data.end(), debug_receive_data);
	stick_data = dualshock2.get_stick_data();
	std::copy(stick_data.begin(), stick_data.end(), debug_stick_data);

	// ADC読み取り
	mcp3208_reader.update(0xF);
	adc_value_array = mcp3208_reader.get();
	std::copy(adc_value_array.begin(), adc_value_array.end(), debug_adc_value_array);

	HAL_Delay(100);

	// ledの更新
	static uint8_t led_count = 0;
	if(led_count > 2){
		HAL_GPIO_TogglePin(LED_01_GPIO_Port, LED_01_Pin);
		led_count = 0;
	}
	led_count++;

}

/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim18){ // メイン処理
		if(HAL_GPIO_ReadPin(sensor[15].port, sensor[15].pin) == GPIO_PIN_RESET){
			maerobo_2022.start();
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_RESET);
		}

		// すべてのピンが set 状態の時に is_expand_completed が true になるようにする
		bool is_expand_completed = true;
		for(int i = 11; i<=14; i++){
			if(HAL_GPIO_ReadPin(sensor[i].port, sensor[i].pin) == GPIO_PIN_SET){
				is_expand_completed = false;
			}
		}
		maerobo_2022.update(is_expand_completed, adc_value_array[0]);


		// md_compare の更新
		state = maerobo_2022.get_state();
		if(state == mb_22::Maerobo_State::WAITING || state == mb_22::Maerobo_State::ENDING){ // 待機時と終了時のみ手動有効
			for (uint8_t i = 0; i < 4; i++) {
				md_compare[i] = stick_data[i]*8;
			}
			md_compare[4] = (receive_data[9] -receive_data[10])*3;
			md_compare[5] = (receive_data[11]-receive_data[12])*3;
			md_compare[6] = (receive_data[17]-receive_data[19])*3;
			md_compare[7] = (receive_data[18]-receive_data[20])*3;
		}else{
			md_compare = maerobo_2022.get_md_compare();
		}
		std::copy(md_compare.begin(), md_compare.end(), debug_md_compare);

		// MDの操作
		sr_data = 0;
		for (uint8_t i=0; i < 8; i++) {
			if(md_compare[i] >= 0){
				__HAL_TIM_SET_COMPARE(md_pwm[i].htim, md_pwm[i].channel, md_compare[i]);
				sr_data += 1<<(i*2);
			}else{
				__HAL_TIM_SET_COMPARE(md_pwm[i].htim, md_pwm[i].channel, -md_compare[i]);
				sr_data += 1<<(i*2+1);
			}
		}
		tc74hc595_writer.update((uint8_t*)&sr_data,  2);

		// ledの更新
		// 点滅
		static uint8_t led_count = 0;
		if(led_count > 2){
			HAL_GPIO_TogglePin(LED_02_GPIO_Port, LED_02_Pin);
			led_count = 0;
		}
		led_count++;
		// state インジゲータ
		for (int i = 3; i < 8; ++i) {
			HAL_GPIO_WritePin(led[i].port, led[i].pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(led[(uint8_t)state+2].port, led[(uint8_t)state+2].pin, GPIO_PIN_SET);
	}
}
/* Function Body End */
