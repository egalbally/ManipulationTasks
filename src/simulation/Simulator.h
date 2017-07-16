#ifndef CS225A_SIMULATOR_H
#define CS225A_SIMULATOR_H

// SAI
#include <model/ModelInterface.h>
#include <simulation/SimulationInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// Standard
#include <string>
#include <thread>

// External
#include <Eigen/Core>

struct SimulatorRobot {

	const std::string kRedisKeyPrefix = "cs225a::";
	const std::string KEY_INTERACTION_COMMAND_TORQUES;
	const std::string KEY_COMMAND_TORQUES;
	const std::string KEY_JOINT_POSITIONS;
	const std::string KEY_JOINT_VELOCITIES;
	const std::string KEY_TIMESTAMP;

	const std::shared_ptr<Model::ModelInterface> robot_;
	const std::string robot_name_;

	SimulatorRobot(std::shared_ptr<Model::ModelInterface> robot, const std::string& robot_name) :
		KEY_INTERACTION_COMMAND_TORQUES(kRedisKeyPrefix + robot_name + "::actuators::fgc_interact"),
		KEY_COMMAND_TORQUES            (kRedisKeyPrefix + robot_name + "::actuators::fgc"),
		KEY_JOINT_POSITIONS            (kRedisKeyPrefix + robot_name + "::sensors::q"),
		KEY_JOINT_VELOCITIES           (kRedisKeyPrefix + robot_name + "::sensors::dq"),
		KEY_TIMESTAMP                  (kRedisKeyPrefix + robot_name + "::timestamp"),
		robot_(robot),
		robot_name_(robot_name)
	{
		robot->_q.setZero();
		robot->_dq.setZero();
	}

};

class Simulator {

public:

	Simulator(std::shared_ptr<Simulation::SimulationInterface> sim,
	          const std::vector<std::shared_ptr<Model::ModelInterface>>& robots,
	          const std::vector<std::string>& robot_names) :
		sim_(sim)
	{
		for (int i = 0; i < robots.size(); i++) {
			robots_.emplace_back(robots[i], robot_names[i]);
		}
	}

	/***** Constants *****/

	const double kSensorWriteFreq = 1e3;
	const double kSimulationFreq = 1e4;

	const std::string kRedisHostname = "127.0.0.1";
	const int kRedisPort = 6379;

	/***** Member functions *****/

	void run();

	/***** Member variables *****/

	const std::shared_ptr<Simulation::SimulationInterface> sim_;
	std::vector<struct SimulatorRobot> robots_;

	LoopTimer timer_;
	RedisClient redis_;

};

#endif  // CS225A_SIMULATOR_H
