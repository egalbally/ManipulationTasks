#include "DemoProject.h"

#include <iostream>
#include <fstream>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

/**
 * DemoProject::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void DemoProject::readRedisValues() {
	// Read from Redis current sensor values
	robot->_q = redis_.getEigenMatrix(KEY_JOINT_POSITIONS);
	robot->_dq = redis_.getEigenMatrix(KEY_JOINT_VELOCITIES);

	// Get current simulation timestamp from Redis
	// t_curr_ = stod(redis_.get(KEY_TIMESTAMP));

	// Read in KP and KV from Redis (can be changed on the fly in Redis)
	kp_pos_ = stod(redis_.get(KEY_KP_POSITION));
	kv_pos_ = stod(redis_.get(KEY_KV_POSITION));
	kp_ori_ = stod(redis_.get(KEY_KP_ORIENTATION));
	kv_ori_ = stod(redis_.get(KEY_KV_ORIENTATION));
	kp_joint_ = stod(redis_.get(KEY_KP_JOINT));
	kv_joint_ = stod(redis_.get(KEY_KV_JOINT));
	kp_joint_init_ = stod(redis_.get(KEY_KP_JOINT_INIT));
	kv_joint_init_ = stod(redis_.get(KEY_KV_JOINT_INIT));
	kp_screw_ = stod(redis_.get(KEY_KP_SCREW));
	kv_screw_ = stod(redis_.get(KEY_KV_SCREW));
	kp_sliding_ = stod(redis_.get(KEY_KP_SLIDING));
	kp_bias_ = stod(redis_.get(KEY_KP_BIAS));

	Eigen::VectorXd F_sensor_6d = redis_.getEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE);
	
	// Offset moment bias
	F_sensor_6d(3) += 0.2;

	// Transform sensor measurements to EE frame
	R_sensor_to_ee_ << -1/sqrt(2), -1/sqrt(2), 0,
	                   1/sqrt(2), -1/sqrt(2), 0,
	                   0, 		   0, 		 -1;

	F_sensor_ = R_sensor_to_ee_ * F_sensor_6d.head(3);
	M_sensor_ = R_sensor_to_ee_ * F_sensor_6d.tail(3);

	// Set moments to zero when they are outside of a range to avoid vibrations
	for (int i = 0; i<3;i++){
		if (M_sensor_(i) < 0.13 && M_sensor_(i) > -0.13){
			M_sensor_(i) = 0;
		}
	}
	
	Eigen::VectorXd F_controller(6); //forces in EE and capped moments in EE
	F_controller << F_sensor_, M_sensor_;

	redis_.setEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE + "_controller", F_controller);
}

/**
 * DemoProject::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void DemoProject::writeRedisValues() {
	// Send end effector position and desired position
	redis_.setEigenMatrix(KEY_EE_POS, x_);
	redis_.setEigenMatrix(KEY_EE_POS_DES, x_des_);

	// Send torques
	redis_.setEigenMatrix(KEY_COMMAND_TORQUES, command_torques_);
}

/**
 * DemoProject::updateModel()
 * --------------------------
 * Update the robot model and all the relevant member variables.
 */
void DemoProject::updateModel() {
	// Update the model
	robot->updateModel();

	// Forward kinematics
	robot->position(x_, "link6", Eigen::Vector3d::Zero());
	robot->rotation(R_ee_to_base_, "link6");
	robot->linearVelocity(dx_, "link6", Eigen::Vector3d::Zero());
	robot->angularVelocity(w_, "link6");

	// Jacobians
	robot->J_0(J_, "link6", Eigen::Vector3d::Zero());
	robot->Jv(Jv_, "link6", Eigen::Vector3d::Zero());
	robot->nullspaceMatrix(N_, J_);
	robot->nullspaceMatrix(Nv_, Jv_);

	// Dynamics
	robot->taskInertiaMatrixWithPseudoInv(Lambda_x_, Jv_);
	robot->taskInertiaMatrixWithPseudoInv(Lambda_, J_);
	robot->gravityVector(g_);
}

/**
 * DemoProject::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
DemoProject::ControllerStatus DemoProject::computeJointSpaceControlTorques() {
	try {
		int flag = stoi(redis_.get(KEY_UI_FLAG));
		if (flag) return FINISHED;	
	} catch (std::exception& e) {
		cout << e.what() << endl;
	}
	return RUNNING;

	// Joint space velocity saturation
	Eigen::VectorXd q_err = robot->_q - q_des_;
	dq_des_ = -(kp_joint_init_ / kv_joint_init_) * q_err;
	double v = kMaxVelocity / dq_des_.norm();
	if (v > 1) v = 1;
	Eigen::VectorXd dq_err = robot->_dq - v * dq_des_;
	std::cout << q_err.transpose() << " " << q_err.norm() << " " << robot->_dq.norm() << std::endl;

	// Angular momentum after initialization
	Eigen::Vector3d w_init;
	w_init = w_;

	// Finish if the robot has converged to q_initial
	if (q_err.norm() < kToleranceInitQ && robot->_dq.norm() < kToleranceInitDq) {
		return FINISHED;
	}

	// Compute torques
	Eigen::VectorXd ddq = -kv_joint_init_ * dq_err;
	command_torques_ = robot->_M * ddq;
	return RUNNING;
}

/**
 * DemoProject::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::computeOperationalSpaceControlTorques() {
	// PD position control with velocity saturation
	Eigen::Vector3d x_err = x_ - x_des_;
	// Eigen::Vector3d dx_err = dx_ - dx_des_;
	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err_;
	dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
	double v = kMaxVelocity / dx_des_.norm();
	if (v > 1) v = 1;
	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	Eigen::Vector3d ddx = -kv_pos_ * dx_err;

	// Nullspace posture control and damping
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

	// Control torques
	Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::VectorXd F_posture = robot->_M * ddq;
	command_torques_ = Jv_.transpose() * F_x + N_.transpose() * F_posture;

	return RUNNING;
}

/**
 * DemoProject::alignBottleCap()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::alignBottleCap() {
	// Position - set xdes below the current position in z to apply a constant downward force
	Eigen::Vector3d x_des_ee(0,0,0.025);
	Eigen::Vector3d x_bias(0,0.005,0);

	// robot->position(x_des_, "link6", x_des_ee);
	Eigen::Vector3d sliding_vector;
	sliding_vector = x_des_ee.cross(M_sensor_);
	robot->position(x_des_, "link6", x_des_ee + kp_sliding_ * sliding_vector);
	// if (sliding_vector.norm() != 0){
	// 	// sliding_vector = sliding_vector/sliding_vector.norm();
	// }
	// x_des_ += kp_sliding_ * sliding_vector;
	x_des_ += kp_bias_ * x_bias ;

	Eigen::Vector3d x_err = x_ - x_des_;
	Eigen::Vector3d dx_err = dx_ - dx_des_;

	Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

	// Orientation
	Eigen::Vector3d dPhi;
	// if (M_sensor_.norm() ==0){ //aply a constant "w"
		// dPhi.setZero();
	// }else{
	// 	dPhi = ((-R_ee_to_base_ * M_sensor_)/M_sensor_.norm())/2; //about 30 deg per second
	// }
	dPhi = -R_ee_to_base_ * M_sensor_;
	Eigen::Vector3d dw = -kp_ori_ * dPhi - kv_ori_ * w_;
	Eigen::VectorXd ddxdw(6);
	ddxdw << ddx, dw;

	// Nullspace damping	
	Eigen::VectorXd ddq = -kv_joint_ * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq; 

	// Control torques
	// Eigen::Vector3d F_x = Lambda_x_ * ddx;
	// command_torques_ = Jv_.transpose() * F_x + N_.transpose() * F_joint;
	Eigen::VectorXd F_xw = Lambda_ * ddxdw;
	// cout << F_xw.transpose() << endl << endl;

	command_torques_ = J_.transpose() * F_xw + N_.transpose() * F_joint;

	// Finish if sensed moments and angular velocity are zero
	if ((M_sensor_.norm() <= 0.1) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0)) return FINISHED;

	return RUNNING;
}

/**
 * DemoProject::rewindBottleCap()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::checkAlignment() {

	// Finish if sensed moments and angular velocity are zero
	if (!((M_sensor_.norm() <= 0.1) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0))) return FAILED;

	double t_curr = timer_.elapsedTime();
	if (t_curr - t_alignment_ >= kAlignmentWait) return FINISHED;

	return RUNNING;
}

/**
 * DemoProject::rewindBottleCap()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::rewindBottleCap() {
	// Position - set xdes below the current position in z to apply a constant downward force
	robot->position(x_des_, "link6", Eigen::Vector3d(0,0,0.05));
	Eigen::Vector3d x_err = x_ - x_des_;
	Eigen::Vector3d dx_err = dx_ - dx_des_;
	Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;
	
	// Finish if the robot has converged to the last joint limit (+15deg)
	Eigen::VectorXd q_err = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	q_err(6) = robot->_q(6) - (-KukaIIWA::JOINT_LIMITS(6) + 15.0 * M_PI / 180.0);
	if (q_err.norm() < 0.1) return FINISHED;

	//Joint space velocity saturation
	dq_des_ = -(kp_screw_ / kv_screw_) * q_err;
	double v = kMaxVelocity / dq_des_.norm();
	if (v > 1) v = 1;
	Eigen::VectorXd dq_err = robot->_dq - v * dq_des_;
	Eigen::VectorXd ddq = -kv_screw_ * dq_err;

	// Control torques with null space damping
	Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::VectorXd F_joint = robot->_M * ddq;
	command_torques_ = Jv_.transpose() * F_x + Nv_.transpose() * F_joint;

	return RUNNING;
}

/**
 * DemoProject::screwBottleCap()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::screwBottleCap() {
	// Position - set xdes below the current position in z to apply a constant downward force
	robot->position(x_des_, "link6", Eigen::Vector3d(0,0,0.05));
	Eigen::Vector3d x_err = x_ - x_des_;
	Eigen::Vector3d dx_err = dx_ - dx_des_;
	Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

	// Nullspace joint position control and damping
	Eigen::VectorXd q_err = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	q_err(6) = robot->_q(6) - (KukaIIWA::JOINT_LIMITS(6) - 15.0 * M_PI / 180.0);
	dq_des_ = -(kp_screw_ / kv_screw_) * q_err;
	double v = kMaxVelocity / dq_des_.norm(); //velocity saturation
	if (v > 1) v = 1;
	Eigen::VectorXd dq_err = robot->_dq - v * dq_des_;
	Eigen::VectorXd ddq = -kv_screw_ * dq_err;

	// Control torques
	Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::VectorXd F_joint = robot->_M * ddq;
	command_torques_ = Jv_.transpose() * F_x + Nv_.transpose() * F_joint;

	return RUNNING;
}

/**
 * public DemoProject::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void DemoProject::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	redis_.connect(kRedisHostname, kRedisPort);

	// Set gains in Redis if not initialized
	redis_.set(KEY_KP_POSITION, to_string(kp_pos_));
	redis_.set(KEY_KV_POSITION, to_string(kv_pos_));
	redis_.set(KEY_KP_ORIENTATION, to_string(kp_ori_));
	redis_.set(KEY_KV_ORIENTATION, to_string(kv_ori_));
	redis_.set(KEY_KP_JOINT, to_string(kp_joint_));
	redis_.set(KEY_KV_JOINT, to_string(kv_joint_));
	redis_.set(KEY_KP_JOINT_INIT, to_string(kp_joint_init_));
	redis_.set(KEY_KV_JOINT_INIT, to_string(kv_joint_init_));
	redis_.set(KEY_KP_SCREW, to_string(kp_screw_));
	redis_.set(KEY_KV_SCREW, to_string(kv_screw_));
	redis_.set(KEY_UI_FLAG, to_string(0));
	redis_.set(KEY_KP_SLIDING, to_string(kp_sliding_));
	redis_.set(KEY_KP_BIAS, to_string(kp_bias_));

}

/**
 * public DemoProject::runLoop()
 * -----------------------------
 * DemoProject state machine
 */
void DemoProject::runLoop() {
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer_.waitForNextLoop();
		++controller_counter_;

		// Get latest sensor values from Redis and update robot model
		try {
			readRedisValues();
		} catch (std::exception& e) {
			if (controller_state_ != REDIS_SYNCHRONIZATION) {
				std::cout << e.what() << " Aborting..." << std::endl;
				break;
			}
			std::cout << e.what() << " Waiting..." << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
		}
		updateModel();

		switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
			case REDIS_SYNCHRONIZATION:
				if (isnan(robot->_q)) continue;
				cout << "Redis synchronized. Switching to joint space controller." << endl;
				controller_state_ = JOINT_SPACE_INITIALIZATION;
				break;

			// Initialize robot to default joint configuration
			case JOINT_SPACE_INITIALIZATION:
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "Joint position initialized. Switching to align bottle cap." << endl;
					controller_state_ = DemoProject::ALIGN_BOTTLE_CAP;
					q_des_ = robot->_q;
					q_des_(6) = KukaIIWA::JOINT_LIMITS(6);
					// cout << "w" <<endl;
					// cout << w_ <<endl;
				};
				break;
			// Control end effector to desired position
			case ALIGN_BOTTLE_CAP:
				if (alignBottleCap() == FINISHED) {
					cout << "Bottle cap aligned. Switching to check alignment." << endl;
					controller_state_ = CHECK_ALIGNMENT;
					t_alignment_ = timer_.elapsedTime();
				}
				break;
			// Control end effector to desired position
			case CHECK_ALIGNMENT:
				switch (checkAlignment()) {
					case FINISHED:
						cout << "Bottle cap aligned. Switching to rewind cap." << endl;
						controller_state_ = REWIND_BOTTLE_CAP;
						break;
					case FAILED:
						cout << "Bottle cap not aligned. Switching back to align bottle cap." << endl;
						controller_state_ = ALIGN_BOTTLE_CAP;
						break;
					default:
						break;
				}
				break;
			// Control end effector to desired position
			case REWIND_BOTTLE_CAP:
				if (rewindBottleCap() == FINISHED) {
					cout << "Bottle cap rewound. Switching to screw bottle cap." << endl;
					controller_state_ = SCREW_BOTTLE_CAP;
				}
				break;

			// Control end effector to desired position
			case SCREW_BOTTLE_CAP:
				screwBottleCap();
				break;

			// Invalid state. Zero torques and exit program.
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				g_runloop = false;
				command_torques_.setZero();
				break;
		}

		// Check command torques before sending them
		if (isnan(command_torques_)) {
			// cout << "NaN command torques. Sending zero torques to robot." << endl;
			command_torques_.setZero();
		}

		// Send command torques
		writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques_.setZero();
	redis_.setEigenMatrix(KEY_COMMAND_TORQUES, command_torques_);
}

int main(int argc, char** argv) {

	// Parse command line
	if (argc != 4) {
		cout << "Usage: demo_app <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// Argument 0: executable name
	// Argument 1: <path-to-world.urdf>
	string world_file(argv[1]);
	// Argument 2: <path-to-robot.urdf>
	string robot_file(argv[2]);
	// Argument 3: <robot-name>
	string robot_name(argv[3]);

	// Set up signal handler
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Load robot
	cout << "Loading robot: " << robot_file << endl;
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// Start controller app
	cout << "Initializing app with " << robot_name << endl;
	DemoProject app(move(robot), robot_name);
	app.initialize();
	cout << "App initialized. Waiting for Redis synchronization." << endl;
	app.runLoop();

	return 0;
}

// -0.628625
//  0.693623
//   -7.2122
// -0.233478
// -0.218972
// 0.0109215

// 0.0347393
//  -1.30462  We need this
//  -7.07576
//  0.224118
// -0.108637
// 0.0133971

