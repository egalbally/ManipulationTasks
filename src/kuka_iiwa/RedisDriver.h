/**
* \copyright
* TaCo - Task-Space Control Library.<br>
* Copyright (c) 2015-2016
*<br>
* TaCo is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*<br>
* TaCo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*<br>
* You should have received a copy of the Lesser GNU Lesser General Public License
* along with TaCo.  If not, see <http://www.gnu.org/licenses/>.
<br>
Author: Brian Soe <bsoe@stanford.edu>
Author: Mikael Jorda <mjorda@stanford.edu>
Author: Toki Migimatsu <takatoki@stanford.edu>
*/

#ifndef KUKA_IIWA_REDIS_DRIVER_H
#define KUKA_IIWA_REDIS_DRIVER_H

#include "friLBRClient.h"
#include "ButterworthFilter.h"
#include "redis/RedisClient.h"

#include <string>

#include <Eigen/Core>

#ifdef USE_KUKA_LBR_DYNAMICS
	#include <KukaLBRDynamics/Robot.h>
	#include <rbdl/Model.h>
	#ifndef RBDL_BUILD_ADDON_URDFREADER
		#error "Error: RBDL addon urdfmodel not enabled."
	#endif
	#include <rbdl/addons/urdfreader/urdfreader.h>
#endif  // USE_KUKA_LBR_DYNAMICS

namespace KukaIIWA {

/********************
 * Public Constants *
 ********************/

// Default Kuka IIWA port
const int DEFAULT_PORT = 30200;

// Kuka degrees of freedom
const int DOF = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;

// Path to the urdf model and tool file
const char MODEL_FILENAME[] = "resources/kuka_iiwa_driver/kuka_iiwa.urdf";
const char TOOL_FILENAME[]  = "resources/kuka_iiwa_driver/tool.xml";

const std::string KEY_PREFIX = "cs225a::kuka_iiwa::";
// Redis keys sent to robot
const std::string KEY_COMMAND_TORQUES         = KEY_PREFIX + "actuators::fgc";
const std::string KEY_DESIRED_JOINT_POSITIONS = KEY_PREFIX + "actuators::q_des";

// Redis keys returned by robot
const std::string KEY_SENSOR_TORQUES   = KEY_PREFIX + "sensors::torques";
const std::string KEY_JOINT_POSITIONS  = KEY_PREFIX + "sensors::q";
const std::string KEY_JOINT_VELOCITIES = KEY_PREFIX + "sensors::dq";

// Safety limits (for initialization purposes only - use Eigen mappings instead)
// TODO: Find real velocity and jerk limits
const double _ARR_JOINT_LIMITS[KukaIIWA::DOF]    = {2.9670, 2.0944, 2.9670, 2.0944, 2.9670, 2.0944, 3.0543};

const double _ARR_VELOCITY_LIMITS[KukaIIWA::DOF] = {90.0, 90.0, 95.0, 125.0, 135.0, 170.0, 170.0};
// const double ARR_VELOCITY_LIMITS[KukaIIWA::DOF] = {98.0, 98.0, 100.0, 130.0, 140.0, 180.0, 180.0}; // for LBR IIWA 7kg from specs

const double _ARR_TORQUE_LIMITS[KukaIIWA::DOF]   = {176.0, 176.0, 110.0, 110.0, 110.0, 40.0, 40.0}; // LBR iiwa 7 R800
// const double _ARR_TORQUE_LIMITS[KukaIIWA::DOF]   = {320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0}; // LBR iiwa 14 R820

const double _ARR_JERK_LIMITS[KukaIIWA::DOF]     = {8.8, 8.8, 5.5, 5.5, 5.5, 2.0, 2.0};

// Eigen mappings to safety limits
const Eigen::ArrayXd JOINT_LIMITS    = Eigen::Array<double,DOF,1>(_ARR_JOINT_LIMITS) - 10.0 * M_PI/180.0;
const Eigen::ArrayXd TORQUE_LIMITS   = Eigen::Array<double,DOF,1>(_ARR_TORQUE_LIMITS);
const Eigen::ArrayXd VELOCITY_LIMITS = Eigen::Array<double,DOF,1>(_ARR_VELOCITY_LIMITS) * M_PI/180.0;
const Eigen::ArrayXd JERK_LIMITS     = Eigen::Array<double,DOF,1>(_ARR_JERK_LIMITS);

// Height limits for the wrist [low high]
const double POS_WRIST_LIMITS[2] = {0.45, 1.05};

}

namespace KUKA {
namespace FRI {

/********************************
 * RedisDriver Class Definition *
 ********************************/

/**
 * \brief Client for Kuka LBR IIWA that reads and writes to shared memory.
 */
class RedisDriver : public KUKA::FRI::LBRClient
{

public:

	RedisDriver(const std::string& redis_ip=RedisServer::DEFAULT_IP,
	            const int redis_port=RedisServer::DEFAULT_PORT);

	/**
	 * \brief Callback that is called whenever the FRI session state changes.
	 *
	 * @param oldState previous FRI session state
	 * @param newState current FRI session state
	 */
	void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

	/**
	 * \brief Callback for the FRI session state 'Commanding Wait'.
	 */
	void waitForCommand();

	/**
	 * \brief Callback for the FRI session state 'Commanding'.
	 */
	void command();

protected:

	/***** Constants *****/

	// Kv for damped exit
	const double arr_kExitKv[KukaIIWA::DOF] = {8, 8, 5, 5, 5, 2, 2};
	const Eigen::ArrayXd kExitKv = Eigen::Array<double,7,1>(arr_kExitKv);

	// Cutoff frequency for velocity filter, in the range of (0, 0.5) of sampling frequency
	const double kCutoffFreq = 0.1;

	/***** State Variables *****/

	// Desired joint positions (for joint space control)
	double arr_q_des_[KukaIIWA::DOF] = {0};
	Eigen::Map<Eigen::VectorXd> q_des_ = Eigen::Map<Eigen::VectorXd>(arr_q_des_, KukaIIWA::DOF);

	// Desired joint torques (for torque control)
	double arr_command_torques_[KukaIIWA::DOF] = {0};
	Eigen::Map<Eigen::VectorXd> command_torques_ = Eigen::Map<Eigen::VectorXd>(arr_command_torques_, KukaIIWA::DOF);

	// Sensor joint positions
	double arr_q_[KukaIIWA::DOF] = {0};
	Eigen::Map<Eigen::VectorXd> q_ = Eigen::Map<Eigen::VectorXd>(arr_q_, KukaIIWA::DOF);

	// Sensor joint velocities
	double arr_dq_[KukaIIWA::DOF] = {0};
	Eigen::Map<Eigen::VectorXd> dq_ = Eigen::Map<Eigen::VectorXd>(arr_dq_, KukaIIWA::DOF);

	// Sensor joint torques
	double arr_sensor_torques_[KukaIIWA::DOF] = {0};
	Eigen::Map<Eigen::VectorXd> sensor_torques_ = Eigen::Map<Eigen::VectorXd>(arr_sensor_torques_, KukaIIWA::DOF);

	// Dither in position to activate friction compensation
	double arr_dither_[LBRState::NUMBER_OF_JOINTS] = {0};

	// Filtered joint velocities
	Eigen::VectorXd dq_filtered_ = Eigen::VectorXd::Zero(KukaIIWA::DOF);

	// Previous joint positions
	Eigen::VectorXd q_prev_ = Eigen::VectorXd::Zero(KukaIIWA::DOF);

	// Previous command torques
	Eigen::VectorXd command_torques_prev_ = Eigen::VectorXd::Zero(KukaIIWA::DOF);

	/***** Misc Member Variables *****/

	// Redis client
	RedisClient redis_;

	// Velocity filter
	sai::ButterworthFilter velocity_filter_;

	// Time of previous measurement, for calculating dt and velocity
	timespec t_prev_ = {0, 0};

	// Last fri command mode, for detecting mode change
	KUKA::FRI::EClientCommandMode fri_command_mode_ = KUKA::FRI::NO_COMMAND_MODE;

	// Flag to go to damped exit and number of ticks left for exit
	bool exit_program_ = false;
	int exit_counter_ = 40;

	// To compute the sine wave for dither and friction compensation
	double t_sample_ = 0.0;
	double t_curr_ = 0.0;

	// Number of driver iterations
	unsigned long long num_iters_ = 0;

#ifdef USE_KUKA_LBR_DYNAMICS
	/**
 	 * \brief Parse tool.xml file.
	 */
	void parseTool();

	// Kuka dynamics
	kuka::Robot dynamics_;

	// RBDL dynamics for wrist safety implementation
	RigidBodyDynamics::Model rbdl_model_;

	// Tool properties
	double tool_mass_ = 0;
	Eigen::Vector3d tool_com_ = Eigen::Vector3d::Zero();
#endif  // USE_KUKA_LBR_DYNAMICS

};

} //namespace FRI
} //namespace KUKA

#endif  // KUKA_IIWA_REDIS_DRIVER_H
