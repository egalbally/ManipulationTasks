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

#include "SchunkGripperRedisDriver.h"
#include <iostream>

using namespace SchunkGripper;

static volatile bool g_runloop = true;
static void stop(int) { g_runloop = false; }

int main(int argc, char *argv[]) {
	// Set up signal handler
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Usage
	std::cout << "Usage: schunk_gripper_driver [-rs REDIS_SERVER_IP] [-rp REDIS_SERVER_PORT]" << std::endl
	          << std::endl
	          << "This driver provides a Redis interface for communication with the Schunk Gripper." << std::endl
	          << std::endl
	          << "Optional arguments:" << std::endl
	          << "  -rs REDIS_SERVER_IP" << std::endl
	          << "\t\t\t\tRedis server IP (default " << RedisServer::DEFAULT_IP << ")." << std::endl
	          << "  -rp REDIS_SERVER_PORT" << std::endl
	          << "\t\t\t\tRedis server port (default " << RedisServer::DEFAULT_PORT << ")." << std::endl
	          << std::endl;

	// Parse arguments
	std::string redis_ip = RedisServer::DEFAULT_IP;
	int redis_port = RedisServer::DEFAULT_PORT;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-rs")) {
			// Redis server IP
			redis_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-rp")) {
			// Redis server port
			sscanf(argv[++i], "%d", &redis_port);
		}
	}

	// Run driver
	SchunkGripperRedisDriver app(redis_ip, redis_port);
	app.run();
}

SchunkGripperRedisDriver::SchunkGripperRedisDriver(const std::string& redis_ip, const int redis_port) {
	// Redis client
	redis_.connect(redis_ip, redis_port);

	// Loop timer
	timer_.setLoopFrequency(kControlFrequency); // 1 kHz
	timer_.setCtrlCHandler(stop);        // exit while loop on ctrl-c
	timer_.initializeTimer(1000000);     // 1ms pause before starting loop

	// Initialize gripper
	gripper_.open();
	gripper_.start();
	gripper_.doSetForceLimit(kForceLimit);
	gripper_.doSetAcceleration(kAcceleration);

	std::cout << "Schunk Device State: " << gripper_.getSystemState() << std::endl;

	redis_.set(KEY_POSITION_DESIRED, std::to_string(position_desired_));
}

void SchunkGripperRedisDriver::readRedisValues() {
	try {
		position_desired_ = std::stod(redis_.get(KEY_POSITION_DESIRED));
		if (position_desired_ < POSITION_MIN) {
			position_desired_ = POSITION_MIN;
		} else if (position_desired_ > POSITION_MAX) {
			position_desired_ = POSITION_MAX;
		}
	} catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}
}

void SchunkGripperRedisDriver::writeRedisValues() {
	float force = gripper_.getForce();
	redis_.set(KEY_FORCE, std::to_string(force));

	float position = gripper_.getOpeningWidth();
	redis_.set(KEY_POSITION, std::to_string(position));

	SystemState state = static_cast<SystemState>(gripper_.getSystemState());
	redis_.set(KEY_STATE, std::to_string(state));
	switch (state) {
		case SYSTEM_STATE_SCRIPT_FAILURE:
			std::cout << "SCRIPT FAILURE" << std::endl;
			break;
		case SYSTEM_STATE_CMD_FAILURE:
			std::cout << "CMD FAILURE" << std::endl;
			break;
		case SYSTEM_STATE_FINGER_FAULT:
			std::cout << "FINGER_FAULT" << std::endl;
			break;
		case SYSTEM_STATE_CURR_FAULT:
			std::cout << "CURR FAULT" << std::endl;
			break;
		case SYSTEM_STATE_POWER_FAULT:
			std::cout << "POWER FAULT" << std::endl;
			break;
		case SYSTEM_STATE_TEMP_FAULT:
			std::cout << "TEMP FAULT" << std::endl;
			break;
		case SYSTEM_STATE_TEMP_WARNING:
			std::cout << "TEMP WARNING" << std::endl;
			break;
		case SYSTEM_STATE_FAST_STOP:
			std::cout << "FAST_STOP" << std::endl;
			break;
		case SYSTEM_STATE_OVERDRIVE_MODE:
			std::cout << "OVERDRIVE MODE" << std::endl;
			break;
		default:
			break;
	}
}

void SchunkGripperRedisDriver::commandGripper() {
	gripper_.doPrePositionFingers(position_desired_, kSpeed);
}

void SchunkGripperRedisDriver::run() {
	while (g_runloop) {
		timer_.waitForNextLoop();

		readRedisValues();
		writeRedisValues();
		commandGripper();
	}

	// Close gripper
	gripper_.stop();
	gripper_.close();
}