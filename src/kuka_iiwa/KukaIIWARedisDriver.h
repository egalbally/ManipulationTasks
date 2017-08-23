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

#include "KukaIIWA.h"
#include "friLBRClient.h"
#include "ButterworthFilter.h"
#include "redis/RedisClient.h"

#include <string>

#include <Eigen/Core>

#ifdef USE_KUKA_LBR_DYNAMICS
	#include "KukaLBRDynamics/Robot.h"
	#include <rbdl/Model.h>
	#ifndef RBDL_BUILD_ADDON_URDFREADER
		#error "Error: RBDL addon urdfmodel not enabled."
	#endif
	#include <rbdl/addons/urdfreader/urdfreader.h>
#endif  // USE_KUKA_LBR_DYNAMICS

namespace KUKA {
namespace FRI {

/**
 * \brief Client for Kuka LBR IIWA that reads and writes to shared memory.
 */
class KukaIIWARedisDriver : public KUKA::FRI::LBRClient
{

public:

	KukaIIWARedisDriver(const std::string& redis_ip=RedisServer::DEFAULT_IP,
	                    const int redis_port=RedisServer::DEFAULT_PORT,
	                    const char *tool_filename=KukaIIWA::TOOL_FILENAME);

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
	const Eigen::ArrayXd kExitKv = KukaIIWA::VectorXd(8, 8, 5, 5, 5, 2, 2);

	// Cutoff frequency for velocity filter, in the range of (0, 0.5) of sampling frequency
	const double kCutoffFreq = 0.1;

	// Torque offsets
	const Eigen::VectorXd kTorqueOffset = KukaIIWA::VectorXd(-0.5, 1.0, 0.0, -0.7, 0.0, 0.05, 0.0);

#ifdef USE_KUKA_LBR_DYNAMICS
	// Constant end effector properties (without tool.xml)
	const double kMassEE = 0.2;
	const Eigen::Vector3d kCenterOfMassEE = Eigen::Vector3d(0, 0, -0.081);
#endif  // USE_KUKA_LBR_DYNAMICS

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
	void parseTool(const char *tool_filename);

	// Kuka dynamics
	kuka::Robot dynamics_;

	// RBDL dynamics for wrist safety implementation
	RigidBodyDynamics::Model rbdl_model_;

	// Tool properties
	double tool_mass_ = 0;
	Eigen::Vector3d tool_com_ = Eigen::Vector3d::Zero();
#endif  // USE_KUKA_LBR_DYNAMICS

};

}  // namespace KUKA
}  // namespace FRI

#endif  // KUKA_IIWA_REDIS_DRIVER_H
