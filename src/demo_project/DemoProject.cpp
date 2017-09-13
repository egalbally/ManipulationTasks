#include "DemoProject.h"

#include <iostream>

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
 * public DemoProject::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void DemoProject::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Start redis client
	redis_.connect();

	// Set gains in Redis
	for (const auto& key_val : K) {
		redis_.set(KukaIIWA::KEY_PREFIX + "tasks::" + key_val.first, to_string(key_val.second));
	}

	// Set flag in Redis
	redis_.set(KEY_UI_FLAG, to_string(controller_flag_));
}

/**
 * DemoProject::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void DemoProject::readRedisValues() {
	// Read from Redis current sensor values
	robot->_q = redis_.getEigenMatrix(KukaIIWA::KEY_JOINT_POSITIONS);
	robot->_dq = redis_.getEigenMatrix(KukaIIWA::KEY_JOINT_VELOCITIES);

	// Read gains from Redis
	for (auto& key_val : K) {
		key_val.second = stod(redis_.get(KukaIIWA::KEY_PREFIX + "tasks::" + key_val.first));
	}

	// Read flag from Redis
	controller_flag_ = stoi(redis_.get(KEY_UI_FLAG)) > 0;

	// Offset force bias
	Eigen::VectorXd F_sensor_6d = redis_.getEigenMatrix(OptoForce::KEY_6D_SENSOR_FORCE);
	F_sensor_6d -= redis_.getEigenMatrix(OptoForce::KEY_6D_SENSOR_FORCE_BIAS);
	F_sensor_6d = F_sensor_6d_filter_.update(F_sensor_6d);

	// Transform sensor measurements to EE frame
	F_sensor_ = F_sensor_6d.head(3);
	M_sensor_ = F_sensor_6d.tail(3);

	// Publish filtered force to Redis
	redis_.setEigenMatrix(OptoForce::KEY_6D_SENSOR_FORCE + "_controller", F_sensor_6d);

	// Get position of shelf
	pos_shelf_ = redis_.getEigenMatrix(OptiTrack::KEY_POS_RIGID_BODIES) + kBaseToOptiTrackOffset;
	ori_shelf_.coeffs() = redis_.getEigenMatrix(OptiTrack::KEY_ORI_RIGID_BODIES);
}

/**
 * DemoProject::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void DemoProject::writeRedisValues() {
	// Send end effector position and desired position
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::x", x_);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::x_des", x_des_);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::R", R_ee_to_base_);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::R_des", R_des_);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::dPhi", dPhi_);

	// angle between contact surface normal and cap normal
	// redis_.set(KukaIIWA::KEY_PREFIX + "sensor::theta", to_string(theta_));

	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::op_point", op_point_);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::dx", dx_);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::w", w_);

	// Check command torques before sending them
	if (isnan(command_torques_)) {
		cout << "NaN command torques. Sending zero torques to robot." << endl;
		command_torques_.setZero();
	}

	// Send torques
	if (controller_flag_) redis_.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, command_torques_);
}

/**
 * DemoProject::estimatePivotPoint()
 * ----------------------------------------------------
 * Estimate pivot point from the linear and angular velocity.
 */
Eigen::Vector3d DemoProject::estimatePivotPoint() {
	/**
 	 * Assuming the end effector is rotating about a point, we know:
 	 *
 	 *   v = -w x r = r x w
 	 *
 	 * Where r is the vector from the end effector to the pivot point.
 	 * The orthogonal projection of r on w is:
 	 *
 	 *   r_orth = w x (r x w) / |w|^2 = w x v / |w|^2
 	 *
 	 * We can determine the pivot point up to a line parallel to w:
 	 *
 	 *   x_pivot = x_ee + r_orth = x_ee + (w x v) / |w|^2
 	 */
	// Eigen::Vector3d x_pivot = x_ + w_.cross(dx_) / dx_.normSquared();
	// return x_pivot;
	// return Eigen::Vector3d(op_point_filter_.update(x_pivot));

	/**
	 * Given that we are applying only a force, we read reaction forces at the sensor:
	 *
	 *   F_s = F_r
	 *   M_s = r x F_r = r x F_s
	 *
 	 * Where r is the vector from the end effector to the pivot point.
 	 * The orthogonal projection of r on M_s is:
 	 *
 	 *   r_orth = F_s x (r x F_s) / |F_s|^2 = F_s x M_s / |F_s|^2
 	 *
 	 * We can determine the pivot point up to a line parallel to M_s:
 	 *
 	 *   x_pivot = x_ee + r_orth = x_ee + (F_s x M_s) / |F_s|^2
 	 */
	Eigen::Vector3d x_pivot = x_ + F_sensor_.cross(M_sensor_) / F_sensor_.squaredNorm();
	return x_pivot;
	// return Eigen::Vector3d(op_point_filter_.update(x_pivot));
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
	x_  = robot->position("link6", op_point_);
	dx_ = robot->linearVelocity("link6", op_point_);
	w_  = robot->angularVelocity("link6");
	R_ee_to_base_ = robot->rotation("link6");
	// theta_ = acos(abs(F_sensor_.dot(Eigen::Vector3d(0,0,1))) / F_sensor_.norm());

	// Jacobians
	// op_point_ = estimatePivotPoint();
	J_cap_ = robot->J("link6", op_point_);
	Jv_ = robot->Jv("link6", Eigen::Vector3d::Zero());
	Jw_ = robot->Jw("link6");
	Jv_cap_ = robot->Jv("link6", op_point_);

	// Dynamics
	Eigen::MatrixXd Jbar_temp;
	robot->operationalSpaceMatrices(Lambda_cap_, Jbar_temp, N_cap_, J_cap_);
	robot->operationalSpaceMatrices(Lambda_x_, Jbar_temp, Nv_, Jv_);
	robot->operationalSpaceMatrices(Lambda_x_cap_, Jbar_temp, Nv_cap_, Jv_cap_);
	Jw_cap_ = Jw_ * Nv_cap_;
	robot->operationalSpaceMatrices(Lambda_r_cap_, Jbar_temp, Nvw_cap_, Jw_cap_, Nv_cap_);

	// N_cap_ = robot->nullspaceMatrix(J_cap_);
	// Nv_ = robot->nullspaceMatrix(Jv_);
	// Nv_cap_ = robot->nullspaceMatrix(Jv_cap_);
	// // Jw_cap_ = Jw_ * Nv_cap_;
	// // std::cout << Jw_cap_.rows() << " " << Jw_cap_.cols() << "; " << Nv_cap_.rows() << " " << Nv_cap_.cols() << std::endl;
	// Nvw_cap_ = robot->nullspaceMatrix(Jw_cap_, Nv_cap_);

	// // Dynamics
	// Lambda_cap_ = robot->taskInertiaMatrixWithPseudoInv(J_cap_);
	// Lambda_x_ = robot->taskInertiaMatrixWithPseudoInv(Jv_);
	// Lambda_x_cap_ = robot->taskInertiaMatrixWithPseudoInv(Jv_cap_);
	// Lambda_r_cap_ = robot->taskInertiaMatrixWithPseudoInv(Jw_cap_);
	g_ = robot->gravityVector();
}

/**
 * DemoProject::initializeJointSpace()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
DemoProject::ControllerStatus DemoProject::initializeJointSpace() {
	// Joint space velocity saturation
	Eigen::VectorXd q_err = robot->_q - q_des_;
	dq_des_ = -(K["kp_joint_init"] / K["kv_joint_init"]) * q_err;
	double v = kMaxVelocityInit / dq_des_.norm();
	if (v > 1) v = 1;
	Eigen::VectorXd dq_err = robot->_dq - v * dq_des_;

	// Command torques
	Eigen::VectorXd ddq = -K["kv_joint_init"] * dq_err;
	command_torques_ = robot->_M * ddq;

	// Finish if the robot has converged to q_initial
	if (q_err.norm() < kToleranceInitQ && robot->_dq.norm() < kToleranceInitDq) {
		return FINISHED;
	}

	return RUNNING;
}

/**
 * DemoProject::gotoBottleCap()
 * ----------------------------------------------------
 * Go to bottle cap location.
 */
DemoProject::ControllerStatus DemoProject::gotoBottleCap() {
	return FINISHED;
}

/**
 * DemoProject::grabBottleCap()
 * ----------------------------------------------------
 * Grab bottle cap.
 */
DemoProject::ControllerStatus DemoProject::grabBottleCap() {
	gripper_pos_des_ = SchunkGripper::POSITION_MIN;
	gotoBottleCap();

	double t_curr = timer_.elapsedTime();
	return (t_curr - t_init_ >= kGripperWait) ? FINISHED : STABILIZING;
}

/**
 * DemoProject::gotoBottleViaPoint()
 * ----------------------------------------------------
 * Go to bottle via point before approaching for contact.
 */
DemoProject::ControllerStatus DemoProject::gotoBottleViaPoint() {
	// Position
	redis_.setEigenMatrix("a", ori_shelf_.matrix());
	x_des_ = ori_shelf_.matrix() * kContactPositionsInShelf[idx_bottle_] + pos_shelf_;
	Eigen::Vector3d x_err = x_ - x_des_;

	// Velocity saturation
	dx_des_ = -K["kp_pos_free"]/ K["kv_pos_free"] * x_err;
	double v = kMaxVelocity / dx_des_.norm();
	if (v > 1) v = 1;
	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	Eigen::Vector3d ddx = - K["kv_pos_free"] * dx_err;

	// Orientation
	R_des_ = ori_shelf_.matrix() * kContactOrientationsInShelf[idx_bottle_];
	robot->orientationError(dPhi_, R_des_, R_ee_to_base_);
	Eigen::Vector3d dw = -K["kp_ori_free"] * dPhi_ - K["kv_ori_free"] * w_;

	// Nullspace damping	
	Eigen::VectorXd ddq = -K["kv_joint_free"] * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq; 

	// Command torques
	Eigen::VectorXd ddxdw(6);
	ddxdw << ddx, dw;
	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

	// Finish if the robot has converged to desired position
	if (x_err.norm() < 0.01 && dx_.norm() < kToleranceAlignDx) {
		return FINISHED;
	}
}

/**
 * DemoProject::alignInFreeSpace()
 * ----------------------------------------------------
 * Controller to match the orientation of the cap to the rim of the bottle
 */
DemoProject::ControllerStatus DemoProject::alignInFreeSpace() {

	// Position - set xdes to be 3cm above the position of the rim
	Eigen::Vector3d x_offset2Rim; // given desired offset in base frame
	x_des_ = Eigen::Vector3d(0.012592, -0.670408, 0.32966);// + x_offset2Rim;
	x_des_(0) += kSafetyDistance2Rim;

	// robot->position(x_des_, "link6", x_des_ee);
	Eigen::Vector3d x_err = x_ - x_des_;

	// Velocity saturation
	dx_des_ = -K["kp_pos_free"]/ K["kv_pos_free"] * x_err;
	double v = kMaxVelocity / dx_des_.norm();
	if (v > 1) v = 1;
	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	Eigen::Vector3d ddx = - K["kv_pos_free"] * dx_err;

	// Orientation
	R_des_ << 0.647176, -0.289006, -0.705435, -0.416645, -0.909016, -0.009826, -0.638412, 0.300275, -0.708707;
	robot->orientationError(dPhi_, R_des_, R_ee_to_base_);
	Eigen::Vector3d dw = -K["kp_ori_free"] * dPhi_ - K["kv_ori_free"] * w_;

	// Nullspace damping	
	Eigen::VectorXd ddq = -K["kv_joint_free"] * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq; 

	// Command torques
	Eigen::VectorXd ddxdw(6);
	ddxdw << ddx, dw;
	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

	// Finish if the robot has converged to desired position
	if (x_err.norm() < 0.01 && dx_.norm() < kToleranceAlignDx) {
		return FINISHED;
	}

	return RUNNING;
}

/**
 * DemoProject::freeSpace2Contact()
 * ----------------------------------------------------
 * Controller to move down until the cap is slightly below rim by applying an open loop force
 * while maintaining orientation
 */
DemoProject::ControllerStatus DemoProject::freeSpace2Contact() {
	x_des_ = Eigen::Vector3d(0.012592, -0.670408, 0.32966);
	x_des_(0) = x_(0) - 0.02;
	Eigen::Vector3d x_err = x_ - x_des_;
	Eigen::Vector3d dx_err = dx_ - dx_des_;
	Eigen::Vector3d ddx = -10 * x_err - K["kv_pos"] * dx_err;

	// Orientation
	R_des_ << 0.647176, -0.289006, -0.705435, -0.416645, -0.909016, -0.009826, -0.638412, 0.300275, -0.708707;
	robot->orientationError(dPhi_, R_des_, R_ee_to_base_);
	Eigen::Vector3d dw = -K["kp_ori_free"] * dPhi_ - K["kv_ori_free"] * w_;

	// Nullspace damping	
	Eigen::VectorXd ddq = -K["kv_joint_free"] * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq; 

	// Position-orientation combined
	Eigen::VectorXd ddxdw(6);
	ddxdw << ddx, dw;
	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

	// Stabilize contact
	double t_curr = timer_.elapsedTime();
	if (F_sensor_.norm() > 1.0) {
		return (t_curr - t_init_ >= kContactWait) ? FINISHED : STABILIZING;
	}
	t_init_ = t_curr;

	return RUNNING;
}

/**
 * DemoProject::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
// DemoProject::ControllerStatus DemoProject::computeOperationalSpaceControlTorques() {
// 	// PD position control with velocity saturation
// 	Eigen::Vector3d x_err = x_ - x_des_;
// 	// Eigen::Vector3d dx_err = dx_ - dx_des_;
// 	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err_;
// 	dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
// 	double v = kMaxVelocity / dx_des_.norm();
// 	if (v > 1) v = 1;
// 	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
// 	Eigen::Vector3d ddx = -kv_pos_ * dx_err;

// 	// Nullspace posture control and damping
// 	Eigen::VectorXd q_err = robot->_q - q_des_;
// 	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
// 	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

// 	// Control torques
// 	Eigen::Vector3d F_x = Lambda_x_ * ddx;
// 	Eigen::VectorXd F_posture = robot->_M * ddq;
// 	command_torques_ = Jv_.transpose() * F_x + N_.transpose() * F_posture;

// 	return RUNNING;
// }

// /**
//  * DemoProject::alignBottleCap()
//  * ----------------------------------------------------
//  * Controller to move end effector to desired position.
//  */
// DemoProject::ControllerStatus DemoProject::alignBottleCap() {
// 	// Position - set xdes below the current position in z to apply a constant downward force
// 	Eigen::Vector3d x_des_ee(0,0,0.025);
// 	Eigen::Vector3d x_bias(0,0.005,0);
// 	Eigen::Vector3d sliding_vector;
// 	sliding_vector = x_des_ee.cross(M_sensor_);
// 	robot->position(x_des_, "link6", x_des_ee + kp_sliding_ * sliding_vector);
// 	x_des_ += kp_bias_ * x_bias ;

// 	Eigen::Vector3d x_err = x_ - x_des_;
// 	Eigen::Vector3d dx_err = dx_ - dx_des_;

// 	Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

// 	// Orientation
// 	Eigen::Vector3d dPhi;
// 	dPhi = -R_ee_to_base_ * M_sensor_;
// 	Eigen::Vector3d dw = -kp_ori_ * dPhi - kv_ori_ * w_;
// 	Eigen::VectorXd ddxdw(6);
// 	ddxdw << ddx, dw;

// 	// Nullspace damping	
// 	Eigen::VectorXd ddq = -kv_joint_ * robot->_dq;
// 	Eigen::VectorXd F_joint = robot->_M * ddq; 

// 	// Control torques
// 	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
// 	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

// 	// Finish if sensed moments and angular velocity are zero
// 	if ((M_sensor_.norm() <= 0.1) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0)) return FINISHED;

// 	return RUNNING;
// }

// /**
//  * DemoProject::alignBottleCapExponentialDamping()
//  * ----------------------------------------------------
//  * Controller to move end effector to desired position.
//  */
// DemoProject::ControllerStatus DemoProject::alignBottleCapExponentialDamping() {
// 	// Position - set xdes below the current position in z to apply a constant downward force
// 	Eigen::Vector3d x_des_ee(0,0,0.025);
// 	Eigen::Vector3d x_bias(0,0.005,0);
// 	Eigen::Vector3d sliding_vector;
// 	sliding_vector = x_des_ee.cross(M_sensor_);
// 	robot->position(x_des_, "link6", x_des_ee + kp_sliding_ * sliding_vector);
// 	x_des_ += kp_bias_ * x_bias ;

// 	Eigen::Vector3d x_err = x_ - x_des_;
// 	Eigen::Vector3d dx_err = dx_ - dx_des_;

// 	Eigen::Vector3d ddx =  -kp_pos_ * x_err - kv_pos_ * dx_err;

// 	// Orientation
// 	Eigen::Vector3d dPhi = -R_ee_to_base_ * M_sensor_;
// 	Eigen::Vector3d dPhi_dt = dPhi / kControlFreq;
// 	integral_dPhi_ += dPhi_dt;// - vec_dPhi_[idx_vec_dPhi_];
// 	vec_dPhi_[idx_vec_dPhi_] = dPhi_dt;
// 	idx_vec_dPhi_ = (idx_vec_dPhi_ + 1) % kIntegraldPhiWindow;
// 	redis_.setEigenMatrix("cs225a::kuka_iiwa::integral_dPhi", integral_dPhi_);
// 	redis_.setEigenMatrix("cs225a::kuka_iiwa::dPhi", dPhi);

// 	Eigen::Vector3d dw = -(1-exp(-exp_moreSpeed_*theta)) *kp_ori_ * dPhi - (exp(-exp_lessDamping_*theta)*kv_ori_) * w_ - ki_ori_exp_ * integral_dPhi_;
// 	Eigen::VectorXd ddxdw(6);
// 	ddxdw << ddx, dw;

// 	// Nullspace damping	
// 	Eigen::VectorXd ddq = -kv_joint_ * robot->_dq;
// 	Eigen::VectorXd F_joint = robot->_M * ddq; 

// 	// Control torques
// 	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
// 	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

// 	// Finish if sensed moments and angular velocity are zero
// 	if ((M_sensor_.norm() <= 0.1) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0)) return FINISHED;

// 	return RUNNING;
// }

// /**
//  * DemoProject::alignBottleCapSimple()
//  * ----------------------------------------------------
//  * Controller to move end effector to desired position.
//  */
// DemoProject::ControllerStatus DemoProject::alignBottleCapSimple() {
// 	// Position - set xdes in the opposite direction to the contact force to apply a constant F in that direction
// 	Eigen::Vector3d x_des_ee;
// 	if (F_sensor_.norm() < 5) {
// 		x_des_ee = Eigen::Vector3d(0,0,0.025);
// 	} else {
// 		x_des_ee = - 0.025 * (F_sensor_ / F_sensor_.norm());
// 		x_des_ee += Eigen::Vector3d(0,0,0.025);
// 	}

// 	robot->position(x_des_, "link6", x_des_ee);
// 	Eigen::Vector3d x_err = x_ - x_des_;
// 	Eigen::Vector3d dx_err = dx_ - dx_des_;
// 	Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

// 	// Orientation
// 	Eigen::Vector3d dPhi;
// 	dPhi = -R_ee_to_base_ * M_sensor_;
// 	Eigen::Vector3d dw = -kp_ori_ * dPhi - kv_ori_ * w_;

// 	// Nullspace damping	
// 	Eigen::VectorXd ddq = -kv_joint_ * robot->_dq;
// 	Eigen::VectorXd F_joint = robot->_M * ddq; 

// 	// Position-orientation combined
// 	Eigen::VectorXd ddxdw(6);
// 	ddxdw << ddx, dw;
// 	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
// 	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

// 	// Orientation in nullspace of position
// 	// Eigen::Vector3d F_x = Lambda_x_cap_ * ddx;
// 	// Eigen::Vector3d F_r = Lambda_r_cap_ * dw;
// 	// command_torques_ = Jv_cap_.transpose() * F_x + Nv_cap_.transpose() * F_r + Nvw_cap_.transpose() * F_joint;

// 	// Finish if sensed moments and angular velocity are zero
// 	if ((M_sensor_.norm() <= 0.1) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0)) return FINISHED;

// 	return RUNNING;
// }

/**
 * DemoProject::alignBottleCapForce()
 * ----------------------------------------------------
 * Align bottle cap using closed loop force control.
 */
DemoProject::ControllerStatus DemoProject::alignBottleCapForce() {
	static Eigen::Vector3d integral_F_err = Eigen::Vector3d::Zero();
	static Eigen::Vector3d integral_M_err = Eigen::Vector3d::Zero();

	Eigen::Vector3d dx_err = dx_ - dx_des_;
	Eigen::Vector3d w_err = w_;

	// Force and moment error
	Eigen::Vector3d sliding_vector = Eigen::Vector3d(0, 0, 1).cross(M_sensor_);
	Eigen::Vector3d F_des_ee = Eigen::Vector3d(0, 0, 15.0) + K["kp_sliding"] * sliding_vector;
	Eigen::Vector3d F_des = R_ee_to_base_ * F_des_ee;
	Eigen::Vector3d F_err = -R_ee_to_base_ * F_sensor_ - F_des;

	Eigen::Vector3d M_des = R_ee_to_base_ * Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d M_err = -R_ee_to_base_ * M_sensor_ - M_des;

	// Integral error
	if (F_err.norm() > 3) {
		integral_F_err.setZero();
		integral_M_err.setZero();
	} else {
		integral_F_err += F_err / kControlFreq;
		integral_M_err += M_err / kControlFreq;
	}

	// Control force
	Eigen::Vector3d F_x = F_des - K["kp_force"] * F_err - K["ki_force"] * integral_F_err - K["kv_force"] * dx_err;
	// Clip control force to 3x the desired force
	if (F_x.norm() > 3 * F_des.norm()) {
		F_x = 3 * F_des.norm() / F_x.norm() * F_x;
	}
	Eigen::Vector3d F_r = M_des - K["kp_moment"] * M_err - K["ki_moment"] * integral_M_err - K["kv_moment"] * w_;

	// Redis
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::F", F_x);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::F_err", F_err);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::F_err_integral", integral_F_err);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::M", F_r);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::M_err", M_err);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee::M_err_integral", integral_M_err);

	// Nullspace damping
	Eigen::VectorXd ddq = -K["kv_joint"] * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq;

	// Orientation in nullspace of position
	command_torques_ = Jv_cap_.transpose() * F_x + Jw_cap_.transpose() * F_r;// + Nvw_cap_.transpose() * F_joint;

	// Stabilize alignment
	double t_curr = timer_.elapsedTime();
	if (M_sensor_.norm() <= 0.15 && w_.norm() < 0.01 && F_sensor_(2) < -1.0) {
		F_x_ee_ = R_ee_to_base_ * F_x;
		F_x_ee_(0) = 0;
		F_x_ee_(1) = 0;
		return (t_curr - t_init_ >= kAlignmentWait) ? FINISHED : STABILIZING;
	}
	t_init_ = t_curr;

	return RUNNING;
}

/**
 * DemoProject::rewindBottleCap()
 * ----------------------------------------------------
 * Rewind bottle cap.
 */
DemoProject::ControllerStatus DemoProject::rewindBottleCap() {
	// // Position - set xdes below the current position in z to apply a constant downward force
	// robot->position(x_des_, "link6", Eigen::Vector3d(0,0,0.05) + kPosEndEffector);
	// Eigen::Vector3d x_err = x_ - x_des_;
	// Eigen::Vector3d dx_err = dx_ - dx_des_;
	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

	// Finish if the robot has converged to the last joint limit (+15deg)
	double q_screw_err = robot->_q(6) - (-KukaIIWA::JOINT_LIMITS(6) + 15.0 * M_PI / 180.0);

	//Joint space velocity saturation
	double dq_screw_des = -(K["kp_screw"] / K["kv_screw"]) * q_screw_err;
	double v = kMaxVelocityScrew / abs(dq_screw_des);
	if (v > 1) v = 1;
	double dq_screw_err = robot->_dq(6) - v * dq_screw_des;

	Eigen::VectorXd ddq = -K["kv_joint"] * robot->_dq;
	ddq(6) = -K["kv_screw"] * dq_screw_err;

	// Control torques with null space damping
	// Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::Vector3d F_x = R_ee_to_base_ * F_x_ee_;
	Eigen::VectorXd F_joint = robot->_M * ddq;
	command_torques_ = Jv_cap_.transpose() * F_x + Nv_cap_.transpose() * F_joint;

	// Stabilize rewind
	double t_curr = timer_.elapsedTime();
	if (abs(q_screw_err) < 0.1 && w_.norm() < 0.01) {
		return (t_curr - t_init_ >= kAlignmentWait) ? FINISHED : STABILIZING;
	}
	t_init_ = t_curr;

	return RUNNING;
}

/**
 * DemoProject::screwBottleCap()
 * ----------------------------------------------------
 * Screw bottle cap. Go to rewind if no z-torques detected.
 */
DemoProject::ControllerStatus DemoProject::screwBottleCap() {
	// // Position - set xdes below the current position in z to apply a constant downward force
	// robot->position(x_des_, "link6", Eigen::Vector3d(0,0,0.05) + kPosEndEffector);
	// Eigen::Vector3d x_err = x_ - x_des_;
	// Eigen::Vector3d dx_err = dx_ - dx_des_;
	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

	// Finish if the robot has converged to the last joint limit (+15deg)
	double q_screw_err = robot->_q(6) - (KukaIIWA::JOINT_LIMITS(6) - 15.0 * M_PI / 180.0);
	redis_.set(KukaIIWA::KEY_PREFIX + "tasks::q_screw_err", std::to_string(q_screw_err));

	//Joint space velocity saturation
	double dq_screw_des = -(K["kp_screw"] / K["kv_screw"]) * q_screw_err;
	double v = kMaxVelocityScrew / abs(dq_screw_des);
	if (v > 1) v = 1;
	double dq_screw_err = robot->_dq(6) - v * dq_screw_des;

	Eigen::VectorXd ddq = -K["kv_joint"] * robot->_dq;
	ddq(6) = -K["kv_screw"] * dq_screw_err;

	// Control torques
	// Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::Vector3d F_x = R_ee_to_base_ * F_x_ee_;
	Eigen::VectorXd F_joint = robot->_M * ddq;
	command_torques_ = Jv_cap_.transpose() * F_x + Nv_cap_.transpose() * F_joint;

	// Check screw
	double t_curr = timer_.elapsedTime();
	if (abs(q_screw_err) < 0.1) {
		if (M_sensor_(2) < -0.5) {
			return FINISHED;
		}
		return (t_curr - t_init_ >= kAlignmentWait) ? FAILED : STABILIZING;
	}
	t_init_ = t_curr;

	return RUNNING;
}

/**
 * DemoProject::releaseBottleCap()
 * ----------------------------------------------------
 * Release bottle cap.
 */
DemoProject::ControllerStatus DemoProject::releaseBottleCap() {
	gripper_pos_des_ = SchunkGripper::POSITION_MIN;
	double t_curr = timer_.elapsedTime();
	return (t_curr - t_init_ >= kGripperWait) ? FINISHED : STABILIZING;
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

		ControllerStatus status;
		switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
			case REDIS_SYNCHRONIZATION:
				if (isnan(robot->_q) || !controller_flag_) continue;
				cout << "REDIS_SYNCHRONIZATION      => JOINT_SPACE_INITIALIZATION" << endl;
				controller_state_ = JOINT_SPACE_INITIALIZATION;
				break;

			// Initialize robot to default joint configuration - joint space
			case JOINT_SPACE_INITIALIZATION:
				if (initializeJointSpace() == FINISHED) {
					cout << "JOINT_SPACE_INITIALIZATION => ALIGN_FREE_SPACE" << endl;
					controller_state_ = GOTO_BOTTLE_VIA_POINT; //ALIGN_BOTTLE_CAP
				}
				break;

			/***********   GRAB BOTTLE CAP   *************/

			case GOTO_BOTTLE_CAP:
				if (gotoBottleCap() == FINISHED) {
					cout << "GOTO_BOTTLE_CAP            => GRAB_BOTTLE_CAP" << endl;
					controller_state_ = GRAB_BOTTLE_CAP;
					t_init_ = timer_.elapsedTime();
				}
				break;

			case GRAB_BOTTLE_CAP:
				if (grabBottleCap() == FINISHED) {
					cout << "GRAB_BOTTLE_CAP            => GOTO_BOTTLE_VIA_POINT" << endl;
					controller_state_ = GOTO_BOTTLE_VIA_POINT;
				}
				break;

			/***********   APPROACH BOTTLE   *************/

			case GOTO_BOTTLE_VIA_POINT:
				if (gotoBottleViaPoint() == FINISHED) {
					cout << "GOTO_BOTTLE_VIA_POINT      => FREE_SPACE_TO_CONTACT" << endl;
					idx_bottle_++;
					if (idx_bottle_ == kNumBottles) idx_bottle_--;
					// controller_state_ = FREE_SPACE_TO_CONTACT;
				}
				break;

			case ALIGN_FREE_SPACE:
				if (alignInFreeSpace() == FINISHED) {
					cout << "FREE 2 CONTACT" << endl;
					controller_state_ = FREE_SPACE_TO_CONTACT;
				}
				break;

			case FREE_SPACE_TO_CONTACT:
				if (freeSpace2Contact() == FINISHED) {
					cout << "FREE_SPACE_TO_CONTACT      => ALIGN_BOTTLE_CAP" << endl;
					controller_state_ = ALIGN_BOTTLE_CAP;
				}
				break;

			/************   CONTROL CONTACT   **************/

			case ALIGN_BOTTLE_CAP:
				if (alignBottleCapForce() == FINISHED) {  //alignBottleCapExponentialDamping //alignBottleCap //alignBottleCapSimple
					op_point_ << 0, 0, 0.12;
					cout << "ALIGN_BOTTLE_CAP           => REWIND_BOTTLE_CAP" << endl;
					controller_state_ = REWIND_BOTTLE_CAP;
				}
				break;

			case REWIND_BOTTLE_CAP:
				if (rewindBottleCap() == FINISHED) {
					cout << "REWIND_BOTTLE_CAP          => SCREW_BOTTLE_CAP" << endl;
					controller_state_ = SCREW_BOTTLE_CAP;
				}
				break;

			case SCREW_BOTTLE_CAP:
				switch (screwBottleCap()) {
					case FINISHED:
						cout << "Success!! SCREW_BOTTLE_CAP => RELEASE_BOTTLE_CAP" << endl;
						controller_state_ = RELEASE_BOTTLE_CAP;
						t_init_ = timer_.elapsedTime();
						break;
					case FAILED:
						cout << "Fail.     SCREW_BOTTLE_CAP => REWIND_BOTTLE_CAP" << endl;
						controller_state_ = REWIND_BOTTLE_CAP;
						break;
					default:
						break;
				}
				break;

			/************   RELEASE BOTTLE   **************/

			case RELEASE_BOTTLE_CAP:
				if (releaseBottleCap() == FINISHED) {
					cout << "RELEASE_BOTTLE_CAP         => JOINT SPACE INITIALIZATION" << endl;
					controller_state_ = JOINT_SPACE_INITIALIZATION;
					idx_bottle_++;
				}
				break;

			// Invalid state. Zero torques and exit program.
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				g_runloop = false;
				command_torques_.setZero();
				break;
		}

		// Send command torques
		writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques_.setZero();
	redis_.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, command_torques_);
}

int main(int argc, char** argv) {

	// Argument 0: executable name
	// Argument 1: <path-to-world.urdf>
	string world_file = "resources/demo_project/world.urdf";
	// Argument 2: <path-to-robot.urdf>
	string robot_file = "resources/demo_project/kuka_iiwa.urdf";
	// Argument 3: <robot-name>
	string robot_name = "kuka_iiwa";

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
	DemoProject app(move(robot));
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

