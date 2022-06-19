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
/* Include End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
struct Timer{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
};
/* Struct End */

/* Variable Begin */
std::array<uint8_t, 21> receive_data;
std::array<int16_t, 4> stick_data;
uint8_t debug_receive_data[21];
int16_t debug_stick_data[4];
uint16_t sr_data;
const std::vector<Timer> md_pwm = {	{&htim4,  TIM_CHANNEL_4},
									{&htim4,  TIM_CHANNEL_3},
									{&htim15, TIM_CHANNEL_1},
									{&htim15, TIM_CHANNEL_2},
									{&htim12, TIM_CHANNEL_1},
									{&htim12, TIM_CHANNEL_2},
									{&htim13, TIM_CHANNEL_1},
									{&htim14, TIM_CHANNEL_1} };
std::array<int16_t, 8> md_compare;
int16_t debug_md_compare[8];

//Timer md_pwm[2] = {{&htim4, TIM_CHANNEL_1}, {&htim4,  TIM_CHANNEL_2}};
/* Variable End */

/* Class Constructor Begin */
DUALSHOCK2 dualshock2(hspi1, SPI1_SS_GPIO_Port, SPI1_SS_Pin, 0xF);
tc74hc595::TC74HC595 tc74hc595_writer({SR_SI_GPIO_Port, SR_SI_Pin}, {SR_SCK_GPIO_Port, SR_SCK_Pin}, {SR_RCK_GPIO_Port, SR_RCK_Pin});
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
}

void loop(void){
	dualshock2.update(0x30);
	receive_data = dualshock2.get_data_exex();
	std::copy(receive_data.begin(), receive_data.end(), debug_receive_data);
	stick_data = dualshock2.get_stick_data();
	std::copy(stick_data.begin(), stick_data.end(), debug_stick_data);

	for (uint8_t i = 0; i < 4; i++) {
		md_compare[i] = stick_data[i]*8;
	}
	md_compare[4] = (receive_data[9] -receive_data[10])*3;
	md_compare[5] = (receive_data[11]-receive_data[12])*3;
	md_compare[6] = (receive_data[17]-receive_data[19])*3;
	md_compare[7] = (receive_data[18]-receive_data[20])*3;

	std::copy(md_compare.begin(), md_compare.end(), debug_md_compare);

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
	tc74hc595_writer.update((uint8_t*)&sr_data, 2);

//	for ( uint8_t i=0; i<16; i++ ) {
//		sr_data = (1<<i);
//		tc74hc595_writer.update((uint8_t*)&sr_data, 2);
//		HAL_Delay(200);
//	}

	HAL_Delay(200);

	static uint8_t led_count = 0;
	if(led_count > 2){
		HAL_GPIO_TogglePin(LED_01_GPIO_Port, LED_01_Pin);
		led_count = 0;
	}
	led_count++;

}

/* Function Body Begin */
/* Function Body End */
