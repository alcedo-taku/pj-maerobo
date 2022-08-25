/**
 * @file maerobo2022.hpp
 * @brief
 * @date 2022/08/12
 */

#ifndef SRC_MAEROBO2022_HPP_
#define SRC_MAEROBO2022_HPP_

#include <stdint.h>
#include <array>
namespace mb_22 {

enum class Maerobo_State : uint8_t{
	WAITING,
	EXPANSION,
	RELEASING,
	ENDING,
	REDYING,
};

enum Motor_Number : uint8_t{
	EXPAND = 0,
	RELEASE = 3,
};

class Maerobo_2022 {
private:
	Maerobo_State state = Maerobo_State::WAITING;
	std::array<int16_t, 8> md_compare; //
	uint8_t max_md_compare_accel = 3; //
public:
	Maerobo_2022();
	void start();
	void update(bool is_expand_completed, uint16_t claw_angle);
	std::array<int16_t, 8> get_md_compare();
	Maerobo_State get_state();
};

} /* namespace mb_22 */

#endif /* SRC_MAEROBO2022_HPP_ */