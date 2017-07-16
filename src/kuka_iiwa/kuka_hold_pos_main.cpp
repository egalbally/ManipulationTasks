// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "kuka_iiwa/RedisDriver.h"

#include <iostream>
#include <string>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int signal){ g_runloop = false; }

const std::string kWorldFile = "resources/kuka_hold_pos/world.urdf";
const std::string kRobotFile = "resources/kuka_hold_pos/kuka_iiwa.urdf";
const std::string kRobotName = "Kuka-IIWA";

const double arr_kp_joint[KukaIIWA::DOF] = {100, 100, 60, 60, 10, 8, 5};
const double arr_kv_joint[KukaIIWA::DOF] = {20, 20, 10, 10, 5, 3, 1};

unsigned long long controller_counter = 0;

int main() {
	std::cout << "Loading URDF world model file: " << kWorldFile << std::endl;

	// Start redis client
	RedisClient redis;
	redis.connect();

	// Set up signal handler
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Load robot
	auto robot = new Model::ModelInterface(kRobotFile, Model::rbdl, Model::urdf, true);

	// Read from Redis
	robot->_q = redis.getEigenMatrix(KukaIIWA::KEY_JOINT_POSITIONS);
	robot->_dq = redis.getEigenMatrix(KukaIIWA::KEY_JOINT_VELOCITIES);

	// Update the model
	robot->updateModel();

	// Initialize controller variables
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	Eigen::VectorXd q_initial = robot->_q;

	Eigen::DiagonalMatrix<double, KukaIIWA::DOF> kp_joint;
	kp_joint.diagonal() << 100, 100, 60, 60, 10, 8, 5;
	Eigen::DiagonalMatrix<double, KukaIIWA::DOF> kv_joint;
	kv_joint.diagonal() << 20, 20, 10, 10, 5, 3, 1;

	// Create a loop timer
	LoopTimer timer;
	timer.setLoopFrequency(1e3);   // 1 KHz
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop

	// Loop until interrupt
	while (g_runloop) {
		// Wait for next scheduled loop
		timer.waitForNextLoop();

		// Read from Redis
		robot->_q = redis.getEigenMatrix(KukaIIWA::KEY_JOINT_POSITIONS);
		robot->_dq = redis.getEigenMatrix(KukaIIWA::KEY_JOINT_VELOCITIES);

		// Update the model
		robot->updateModel();

		// Joint control
		command_torques = kp_joint * (q_initial - robot->_q) - kv_joint * robot->_dq;

		// Send torques to robot
		redis.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, command_torques);

		controller_counter++;
	}

	// Clear torques
    command_torques.setZero();
    redis.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
