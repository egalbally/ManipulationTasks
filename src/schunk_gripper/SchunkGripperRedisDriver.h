/*
 * Copyright (C) 2017 Stanford Robotics Lab
 * Author: Toki Migimatsu
 *
 * This code was written based on another code written by Mohammad Khansari and Keegan Go
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef SCHUNK_GRIPPER_REDIS_DRIVER_H
#define SCHUNK_GRIPPER_REDIS_DRIVER_H

#include "SchunkGripper.h"

// SAI
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// Gripper Library
#include <rl/hal/WeissWsg50.h>
#include <rl/util/Timer.h>

/*
 * The gripper is meant to be controlled form a separate thread.
 * It watches the given bit and opens or closes to match it.
 */
class SchunkGripperRedisDriver {

public:

	SchunkGripperRedisDriver(const std::string& redis_ip = RedisServer::DEFAULT_IP,
	                         const int redis_port = RedisServer::DEFAULT_PORT);

	void run();

private:

	/***** Private Constants *****/

	const double kControlFrequency = 500;
	const float kForceLimit = 75.0;
	const float kAcceleration = 4.5;
	const float kSpeed = 0.35;

	/***** Private Member Functions *****/

	void readRedisValues();
	void writeRedisValues();
	void commandGripper();

	/***** Private Member Variables *****/

	RedisClient redis_;
	LoopTimer timer_;
	rl::hal::WeissWsg50 gripper_;

	double position_desired_ = SchunkGripper::POSITION_MAX;
};


#endif  // SCHUNK_GRIPPER_REDIS_DRIVER_H
