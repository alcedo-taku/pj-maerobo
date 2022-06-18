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
std::array<uint8_t, 21> receive_data;
std::array<int16_t, 4> stick_data;
uint8_t debug_receive_data[21];
int16_t debug_stick_data[4];
/* Variable End */

/* Class Constructor Begin */
DUALSHOCK2 dualshock2(hspi1, SPI1_SS_GPIO_Port, SPI1_SS_Pin, 0xF);
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
	dualshock2.init();
	dualshock2.reset_stick();
}

void loop(void){
	dualshock2.update(0x20);
	receive_data = dualshock2.get_data_exex();
	std::copy(receive_data.begin(), receive_data.end(), debug_receive_data);
	stick_data = dualshock2.get_stick_data();
	std::copy(stick_data.begin(), stick_data.end(), debug_stick_data);
	HAL_Delay(100);
	static uint8_t led_count = 0;
	if(led_count > 2){
		HAL_GPIO_TogglePin(LED_01_GPIO_Port, LED_01_Pin);
		led_count = 0;
	}
	led_count++;

}

/* Function Body Begin */
/* Function Body End */
