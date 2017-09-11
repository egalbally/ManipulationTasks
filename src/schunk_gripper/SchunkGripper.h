/**
 * SchunkGripper.h
 *
 * Author: Toki Migimatsu
 * Created: September 2017
 */

#ifndef SAI2_SCHUNK_GRIPPER_H
#define SAI2_SCHUNK_GRIPPER_H

#include "redis/RedisClient.h"
#include <string>

namespace SchunkGripper {

const std::string KEY_PREFIX = RedisServer::KEY_PREFIX + "schunk_gripper::";
const std::string KEY_POSITION_DESIRED = KEY_PREFIX + "actuators::pos_des";
const std::string KEY_POSITION         = KEY_PREFIX + "sensors::pos";
const std::string KEY_FORCE            = KEY_PREFIX + "sensors::force";
const std::string KEY_STATE            = KEY_PREFIX + "state";

const double POSITION_MIN = 0.0309; //0.0 //0.006
const double POSITION_MAX = 0.1097; //0.11

enum SystemState { // From rl::hal::WeissWsg50::SystemState
	SYSTEM_STATE_SCRIPT_FAILURE,
	SYSTEM_STATE_SCRIPT_RUNNING,
	SYSTEM_STATE_CMD_FAILURE,
	SYSTEM_STATE_FINGER_FAULT,
	SYSTEM_STATE_CURR_FAULT,
	SYSTEM_STATE_POWER_FAULT,
	SYSTEM_STATE_TEMP_FAULT,
	SYSTEM_STATE_TEMP_WARNING,
	SYSTEM_STATE_FAST_STOP,
	SYSTEM_STATE_OVERDRIVE_MODE,
	SYSTEM_STATE_TARGET_POS_REACHED,
	SYSTEM_STATE_AXIS_STOPPED,
	SYSTEM_STATE_SOFT_LIMIT_PLUS,
	SYSTEM_STATE_SOFT_LIMIT_MINUS,
	SYSTEM_STATE_BLOCKED_PLUS,
	SYSTEM_STATE_BLOCKED_MINUS,
	SYSTEM_STATE_MOVING,
	SYSTEM_STATE_REFERENCED
};

}

#endif  // SAI2_SCHUNK_GRIPPER_H
