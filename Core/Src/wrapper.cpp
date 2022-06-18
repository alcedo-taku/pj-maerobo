#include "wrapper.hpp"

/* Include Begin */
#include "gpio.h"
#include "spi.h"
#include "DUALSHOCK2.hpp"
/* Include End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
/* Struct End */

/* Variable Begin */
//std::array<uint8_t, 21> receive_data;
//std::array<int16_t, 4> stick_data;
//uint8_t debug_receive_data[21];
//int16_t debug_stick_data[4];
/* Variable End */

/* Class Constructor Begin */
//DUALSHOCK2 dualshock2(hspi1, SPI1_SS_GPIO_Port, SPI1_SS_Pin, 0xF);
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
//	dualshock2.init();
//	dualshock2.reset_stick();
}

void loop(void){
//	dualshock2.update(0x20);
//	receive_data = dualshock2.get_data_exex();
//	std::copy(receive_data.begin(), receive_data.end(), debug_receive_data);
//	stick_data = dualshock2.get_stick_data();
//	std::copy(stick_data.begin(), stick_data.end(), debug_stick_data);
//	HAL_Delay(100);
	HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_06_GPIO_Port, LED_06_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_07_GPIO_Port, LED_07_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_08_GPIO_Port, LED_08_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_12_GPIO_Port, LED_12_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_13_GPIO_Port, LED_13_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_14_GPIO_Port, LED_14_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_15_GPIO_Port, LED_15_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_06_GPIO_Port, LED_06_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_07_GPIO_Port, LED_07_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_08_GPIO_Port, LED_08_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_12_GPIO_Port, LED_12_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_13_GPIO_Port, LED_13_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_14_GPIO_Port, LED_14_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_15_GPIO_Port, LED_15_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
}

/* Function Body Begin */
/* Function Body End */
