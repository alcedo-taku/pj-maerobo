/**
 * @file maerobo2022.cpp
 * @brief
 * @date 2022/08/12
 */

#include "Maerobo2022.hpp"
#include "tim.h"

namespace mb_22 {

Maerobo_2022::Maerobo_2022() {
}

void Maerobo_2022::start(){
	switch (state) {
		case Maerobo_State::WAITING:
			state = Maerobo_State::EXPANSION;
			md_compare[Motor_Number::EXPAND] = 0;
			break;
		case Maerobo_State::ENDING:
			state = Maerobo_State::REDYING;
			md_compare[Motor_Number::EXPAND]  = 0;
			md_compare[Motor_Number::RELEASE] = 0;
			break;
	}
}

void Maerobo_2022::update(bool is_expand_completed, uint16_t claw_angle){
	switch (state) {
		case Maerobo_State::WAITING:
			break;
		case Maerobo_State::EXPANSION:
			// 展開
			if (is_expand_completed){
				md_compare[Motor_Number::EXPAND] = 0;
				state = Maerobo_State::CLEARANCE_TIME;
				clearance_time = HAL_GetTick();
			}else{
				int16_t max_compare = 250;
				if(md_compare[Motor_Number::EXPAND] < max_compare){
					md_compare[Motor_Number::EXPAND] += max_md_compare_accel;
				}
			}
			break;
		case Maerobo_State::CLEARANCE_TIME:
			if(clearance_time + 2000 <= HAL_GetTick()){
				state = Maerobo_State::RELEASING;
			}
			break;
		case Maerobo_State::RELEASING:
			// 開放
			if (claw_angle > 28000){
				md_compare[Motor_Number::RELEASE] = 0;
				state = Maerobo_State::ENDING;
			}else{
				int16_t max_compare = 500;
				if(md_compare[Motor_Number::RELEASE] < max_compare){
					md_compare[Motor_Number::RELEASE] += max_md_compare_accel;
				}
			}
			break;
		case Maerobo_State::ENDING:
			clearance_time = HAL_GetTick();
			break;
		case Maerobo_State::REDYING:
			state = Maerobo_State::WAITING;
			// 展開部を元に戻す
			if(clearance_time + 8000 <= HAL_GetTick()){
//			if (is_expand_completed){
				md_compare[Motor_Number::EXPAND] = 0;
			}else{
				int16_t max_compare = -250;
				if(md_compare[Motor_Number::EXPAND] > max_compare){
					md_compare[Motor_Number::EXPAND] -= max_md_compare_accel;
				}
				state = Maerobo_State::REDYING;
			}
			// 開放部を元に戻す
			if (claw_angle < 16000){
				md_compare[Motor_Number::RELEASE] = 0;
			}else{
				int16_t max_compare = -100;
				if(md_compare[Motor_Number::RELEASE] > max_compare){
					md_compare[Motor_Number::RELEASE] -= max_md_compare_accel;
				}
				state = Maerobo_State::REDYING;
			}
			break;
	}
}

std::array<int16_t, 8> Maerobo_2022::get_md_compare(){
	return md_compare;
}

Maerobo_State Maerobo_2022::get_state(){
	return state;
}

} /* namespace mb_22 */
