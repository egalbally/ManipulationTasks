#include "simulation/Simulator.h"

#include <iostream>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

void Simulator::run() {
	// Create a loop timer
	timer_.setLoopFrequency(kSimulationFreq);  // 1 kHz
	timer_.setCtrlCHandler(stop);  // Exit while loop on ctrl-c

	// Start Redis client
	redis_.connect(kRedisHostname, kRedisPort);

	std::vector<std::string> keys_read;
	std::vector<std::pair<std::string, std::string>> keyvals_write;
	for (auto& r : robots_) {
		auto zeros = Eigen::VectorXd::Zero(r.robot_->dof());
		redis_.setEigenMatrix(r.KEY_INTERACTION_COMMAND_TORQUES, zeros);
		redis_.setEigenMatrix(r.KEY_COMMAND_TORQUES, zeros);
		redis_.setEigenMatrix(r.KEY_JOINT_POSITIONS, r.robot_->_q);
		redis_.setEigenMatrix(r.KEY_JOINT_VELOCITIES, r.robot_->_dq);

		keys_read.push_back(r.KEY_INTERACTION_COMMAND_TORQUES);
		keys_read.push_back(r.KEY_COMMAND_TORQUES);

		keyvals_write.emplace_back(r.KEY_JOINT_POSITIONS, "");
		keyvals_write.emplace_back(r.KEY_JOINT_VELOCITIES, "");
		keyvals_write.emplace_back(r.KEY_TIMESTAMP, "");
	}

	auto t_sensor_write = std::chrono::high_resolution_clock::now();
	while (g_runloop) {
		// Wait for next scheduled loop
		timer_.waitForNextLoop();

		// Read command torques from Redis
		auto redis_values = redis_.pipeget(keys_read);
		int i = 0;
		for (auto& r : robots_) {
			Eigen::VectorXd command_torques = RedisClient::decodeEigenMatrix(redis_values[i++]);
			Eigen::VectorXd interaction_command_torques = RedisClient::decodeEigenMatrix(redis_values[i++]);

			sim_->setJointTorques(r.robot_name_, command_torques + interaction_command_torques);
		}

		// Update simulation by 0.1 ms
		sim_->integrate(1.0 / kSimulationFreq);

		// Update models
		for (auto& r : robots_) {
			sim_->getJointPositions(r.robot_name_, r.robot_->_q);
			sim_->getJointVelocities(r.robot_name_, r.robot_->_dq);
			r.robot_->updateModel();
		}
		
		auto t_curr = std::chrono::high_resolution_clock::now();
		if (std::chrono::duration<double>(t_curr - t_sensor_write).count() >= 1.0 / kSensorWriteFreq) {
			// Write joint kinematics to Redis
			i = 0;
			for (auto& r : robots_) {
				keyvals_write[i++].second = RedisClient::encodeEigenMatrix(r.robot_->_q);
				keyvals_write[i++].second = RedisClient::encodeEigenMatrix(r.robot_->_dq);
				keyvals_write[i++].second = std::to_string(timer_.elapsedSimTime());
			}
			redis_.pipeset(keyvals_write);

			t_sensor_write = t_curr;
		}

	}

	// Clean up keys
	for (auto& r : robots_) {
		redis_.del(r.KEY_JOINT_POSITIONS);
		redis_.del(r.KEY_JOINT_VELOCITIES);
	}
}

int main(int argc, char** argv) {
	// Parse command line
	if (argc < 4 || argc % 2 != 0) {
		std::cout << "Usage: simulator <path-to-world.urdf> <path-to-robot-1.urdf> <robot-name-1> ..." << std::endl;
		exit(0);
	}

	// Argument 0: executable name
	// Argument 1: <path-to-world.urdf>
	std::string world_file(argv[1]);

	// Load robots
	std::vector<std::string> robot_names;
	std::vector<std::shared_ptr<Model::ModelInterface>> robots;
	for (int i = 2; i < argc; i += 2) {
		// Argument 2: <path-to-robot.urdf>
		robots.push_back(std::make_shared<Model::ModelInterface>(std::string(argv[i]), Model::rbdl, Model::urdf, false));
		// Argument 3: <robot-name>
		robot_names.emplace_back(argv[i+1]);
	}

	// Set up signal handler
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Load simulation world
	auto sim = std::make_shared<Simulation::SimulationInterface>(world_file, Simulation::sai2simulation, Simulation::urdf, false);

	Simulator app(sim, robots, robot_names);
	app.run();
}
