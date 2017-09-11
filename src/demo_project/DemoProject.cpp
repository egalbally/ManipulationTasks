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
 * DemoProject::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void DemoProject::readRedisValues() {
	// Read from Redis current sensor values
	robot->_q = redis_.getEigenMatrix(KukaIIWA::KEY_JOINT_POSITIONS);
	robot->_dq = redis_.getEigenMatrix(KukaIIWA::KEY_JOINT_VELOCITIES);

	for (auto& key_val : K) {
		key_val.second = stod(redis_.get(KukaIIWA::KEY_PREFIX + "tasks::" + key_val.first));
	}
	// // Read in KP and KV from Redis (can be changed on the fly in Redis)
	// kp_pos_ = stod(redis_.get(KEY_KP_POSITION));
	// kv_pos_ = stod(redis_.get(KEY_KV_POSITION));
	// kp_ori_ = stod(redis_.get(KEY_KP_ORIENTATION));
	// kv_ori_ = stod(redis_.get(KEY_KV_ORIENTATION));
	// kp_joint_ = stod(redis_.get(KEY_KP_JOINT));
	// kv_joint_ = stod(redis_.get(KEY_KV_JOINT));
	// kp_joint_init_ = stod(redis_.get(KEY_KP_JOINT_INIT));
	// kv_joint_init_ = stod(redis_.get(KEY_KV_JOINT_INIT));
	// kp_screw_ = stod(redis_.get(KEY_KP_SCREW));
	// kv_screw_ = stod(redis_.get(KEY_KV_SCREW));
	// // kp_sliding_ = stod(redis_.get(KEY_KP_SLIDING));
	// // kp_bias_ = stod(redis_.get(KEY_KP_BIAS));

	// kp_pos_free_ = stod(redis_.get(KEY_KP_POS_FREE));
	// kv_pos_free_ = stod(redis_.get(KEY_KV_POS_FREE));
	// kp_ori_free_ = stod(redis_.get(KEY_KP_ORI_FREE));
	// kv_ori_free_ = stod(redis_.get(KEY_KV_ORI_FREE));
	// kv_joint_free_ = stod(redis_.get(KEY_KV_JOINT_FREE));

	// // exp_moreSpeed_ = stod(redis_.get(KEY_MORE_SPEED));
	// // exp_lessDamping_ = stod(redis_.get(KEY_LESS_DAMPING));
	// // kp_ori_exp_ = stod(redis_.get(KEY_KP_ORIENTATION_EXP));
	// // kv_ori_exp_ = stod(redis_.get(KEY_KV_ORIENTATION_EXP));
	// // ki_ori_exp_ = stod(redis_.get(KEY_KI_ORIENTATION_EXP));
	// // kp_pos_exp_ = stod(redis_.get(KEY_KP_POSITION_EXP));
	// // kp_force_ = stod(redis_.get(KEY_KP_FORCE));
	// // kv_force_ = stod(redis_.get(KEY_KV_FORCE));
	// // ki_force_ = stod(redis_.get(KEY_KI_FORCE));
	// // kp_moment_ = stod(redis_.get(KEY_KP_MOMENT));
	// // kv_moment_ = stod(redis_.get(KEY_KV_MOMENT));
	// // ki_moment_ = stod(redis_.get(KEY_KI_MOMENT));

	// Offset force bias
	Eigen::VectorXd F_sensor_6d = redis_.getEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE);
	F_sensor_6d -= redis_.getEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE_BIAS);
	F_sensor_6d = F_sensor_6d_filter_.update(F_sensor_6d);

	// Transform sensor measurements to EE frame
	F_sensor_ = F_sensor_6d.head(3);
	M_sensor_ = F_sensor_6d.tail(3);

	// Publish filtered force to Redis
	redis_.setEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE + "_controller", F_sensor_6d);
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

	// angle between contact surface normal and cap normal
	// redis_.set(KukaIIWA::KEY_PREFIX + "sensor::theta", to_string(theta_));

	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::op_point", op_point_);

	// Send torques
	redis_.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, command_torques_);

	// ee orientation
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::ee_ori", R_ee_to_base_);
	
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
	J_cap_ = robot->J("link6", op_point_);
	Jv_ = robot->Jv("link6", Eigen::Vector3d::Zero());
	Jw_ = robot->Jw("link6");
	// op_point_ = estimatePivotPoint();
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
 * DemoProject::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
DemoProject::ControllerStatus DemoProject::computeJointSpaceControlTorques() {
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
	Eigen::Vector3d dPhi;
	Eigen::Matrix3d desired_orientation;
	desired_orientation << 0.647176, -0.289006, -0.705435, -0.416645, -0.909016, -0.009826, -0.638412, 0.300275, -0.708707;
	Eigen::Matrix3d current_orientation = R_ee_to_base_;
	robot->orientationError(dPhi, desired_orientation, current_orientation);
	Eigen::Vector3d dw = -K["kp_ori_free"] * dPhi - K["kv_ori_free"] * w_;

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
	Eigen::Vector3d dPhi;
	Eigen::Matrix3d desired_orientation;
	desired_orientation << 0.647176, -0.289006, -0.705435, -0.416645, -0.909016, -0.009826, -0.638412, 0.300275, -0.708707;
	Eigen::Matrix3d current_orientation = R_ee_to_base_;
	robot->orientationError(dPhi, desired_orientation, current_orientation);
	Eigen::Vector3d dw = -K["kp_ori_free"] * dPhi - K["kv_ori_free"] * w_;

	// Nullspace damping	
	Eigen::VectorXd ddq = -K["kv_joint_free"] * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq; 

	// Position-orientation combined
	Eigen::VectorXd ddxdw(6);
	ddxdw << ddx, dw;
	Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
	command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

	double t_curr = timer_.elapsedTime();

	if (F_sensor_.norm() > 1.0){
		return FINISHED;
	}// else if (t_curr - t_alignment_ >= kAlignmentWait) {
	//	return FAILED;
	//}

	// // Finish if linear and angular velocity are zero
	// if (x_err.norm() < 0.01 && dx_.norm() < kToleranceAlignDx) return FINISHED;

	return RUNNING;
}

/**
 * DemoProject::checkFreeSpaceAlignment()
 * ----------------------------------------------------
 * Check if cap and rim are aligned by checking if applying a horizontal force causes a non zero velocity of the cap
 */
// DemoProject::ControllerStatus DemoProject::checkFreeSpaceAlignment() {

// 	// Position control with velocity saturation
// 	Eigen::Vector3d x_des_ee;
// 	x_des_ee = Eigen::Vector3d(0.025,0,0); //this will generate a constant horizontal force
// 	robot->position(x_des_, "link6", x_des_ee);
// 	Eigen::Vector3d x_err = x_ - x_des_;
// 	dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
// 	double v = kMaxVelocity / dx_des_.norm();
// 	if (v > 1) v = 1;
// 	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
// 	Eigen::Vector3d ddx = -kv_pos_ * dx_err;

// 	// Nullspace posture control and damping
// 	Eigen::VectorXd ddq = -kv_joint_ * robot->_dq;

// 	// Control torques
// 	Eigen::Vector3d F_x = Lambda_x_ * ddx;
// 	Eigen::VectorXd F_joint = robot->_M * ddq; 
// 	command_torques_ = Jv_.transpose() * F_x + N_cap_.transpose() * F_joint;

// 	double t_curr = timer_.elapsedTime();
	

// 	// Failed if the applied horizontal force caused a lateral displacement
// 	std::cout << F_sensor_.norm() << std::endl;
// 	if (F_sensor_.norm() < 1.0){
// 		return RUNNING;
// 	}// else if (t_curr - t_alignment_ >= kAlignmentWait) {
// 	// 	return FAILED;
// 	// }

// 	return FINISHED;
// }

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
 * Controller to move end effector to desired position.
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
	if (F_x.norm() > 3 * F_des.norm()) {
		F_x = 3 * F_des.norm() / F_x.norm() * F_x;
	}
	Eigen::Vector3d F_r = M_des - K["kp_moment"] * M_err - K["ki_moment"] * integral_M_err - K["kv_moment"] * w_;

	// Redis
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::F_x", F_x);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::F_err", F_err);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::F_err_integral", integral_F_err);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::F_r", F_r);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::M_err", M_err);
	redis_.setEigenMatrix(KukaIIWA::KEY_PREFIX + "tasks::M_err_integral", integral_M_err);

	// Nullspace damping
	Eigen::VectorXd ddq = -K["kv_joint"] * robot->_dq;
	Eigen::VectorXd F_joint = robot->_M * ddq;

	// // Position-orientation combined
	// Eigen::VectorXd ddxdw(6);
	// ddxdw << ddx, dw;
	// Eigen::VectorXd F_xw = Lambda_cap_ * ddxdw;
	// command_torques_ = J_cap_.transpose() * F_xw + N_cap_.transpose() * F_joint;

	// Orientation in nullspace of position
	redis_.setEigenMatrix("sai2::kuka_iiwa::tasks::lambda_x_cap", Lambda_x_cap_);
	// Eigen::Vector3d F_x = Lambda_x_cap_ * ddx;
	// Eigen::Vector3d F_r = Lambda_r_cap_ * dw;
	command_torques_ = Jv_cap_.transpose() * F_x + Jw_cap_.transpose() * F_r;// + Nvw_cap_.transpose() * F_joint;

	// Finish if sensed moments and angular velocity are zero
	if ((M_sensor_.norm() <= 0.15) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0)) {
		F_x_ee_ = R_ee_to_base_ * F_x;
		F_x_ee_(0) = 0;
		F_x_ee_(1) = 0;
		return FINISHED;
	}

	return RUNNING;
}


/**
 * DemoProject::checkAlignment()
 * ----------------------------------------------------
 * Controller to check if sensed moments are zero, angular velocity is zero and Fz is smaller than -1.
 */
DemoProject::ControllerStatus DemoProject::checkAlignment() {

	if (!((M_sensor_.norm() <= 0.15) && (w_.norm() < 0.01) && (F_sensor_(2) < -1.0))) return FAILED;

	double t_curr = timer_.elapsedTime();
	if (t_curr - t_alignment_ >= kAlignmentWait) {
		op_point_ <<0,0,0.12;
		return FINISHED;
	}
	return RUNNING;
}


/**
 * DemoProject::rewindBottleCap()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::rewindBottleCap() {
	// // Position - set xdes below the current position in z to apply a constant downward force
	// robot->position(x_des_, "link6", Eigen::Vector3d(0,0,0.05) + kPosEndEffector);
	// Eigen::Vector3d x_err = x_ - x_des_;
	// Eigen::Vector3d dx_err = dx_ - dx_des_;
	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err;

	// Finish if the robot has converged to the last joint limit (+15deg)
	double q_screw_err = robot->_q(6) - (-KukaIIWA::JOINT_LIMITS(6) + 15.0 * M_PI / 180.0);
	if (abs(q_screw_err) < 0.1) return FINISHED;

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

	return RUNNING;
}

/**
 * DemoProject::stabilizeRewind()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::stabilizeRewind() {

	double t_curr = timer_.elapsedTime();
	if (t_curr - t_alignment_ >= kAlignmentWait && w_.norm() < 0.01) return FINISHED;
	
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

	return RUNNING;
}

/**
 * DemoProject::screwBottleCap()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
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
	if (abs(q_screw_err) < 0.1) return FINISHED;

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

	return RUNNING;
}


/**
 * DemoProject::checkScrew()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::checkScrew() {
	
	if (M_sensor_(2) < -0.5)  return FINISHED;
	
	double t_curr = timer_.elapsedTime();
	if (t_curr - t_alignment_ >= kAlignmentWait) {
		op_point_ <<0,0,12;
		return FAILED;
	}

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
	redis_.connect();

	// Set gains in Redis
	for (const auto& key_val : K) {
		redis_.set(KukaIIWA::KEY_PREFIX + "tasks::" + key_val.first, to_string(key_val.second));
	}
	// // Set gains in Redis if not initialized
	// redis_.set(KEY_KP_POSITION, to_string(kp_pos_));
	// redis_.set(KEY_KV_POSITION, to_string(kv_pos_));
	// redis_.set(KEY_KP_ORIENTATION, to_string(kp_ori_));
	// redis_.set(KEY_KV_ORIENTATION, to_string(kv_ori_));
	// redis_.set(KEY_KP_JOINT, to_string(kp_joint_));
	// redis_.set(KEY_KV_JOINT, to_string(kv_joint_));
	// redis_.set(KEY_KP_JOINT_INIT, to_string(kp_joint_init_));
	// redis_.set(KEY_KV_JOINT_INIT, to_string(kv_joint_init_));
	// redis_.set(KEY_KP_SCREW, to_string(kp_screw_));
	// redis_.set(KEY_KV_SCREW, to_string(kv_screw_));
	redis_.set(KEY_UI_FLAG, to_string(0));
	// // redis_.set(KEY_KP_SLIDING, to_string(kp_sliding_));
	// // redis_.set(KEY_KP_BIAS, to_string(kp_bias_));

	// redis_.set(KEY_KP_POS_FREE, to_string(kp_pos_free_));
	// redis_.set(KEY_KV_POS_FREE, to_string(kv_pos_free_));
	// redis_.set(KEY_KP_ORI_FREE, to_string(kp_ori_free_));
	// redis_.set(KEY_KV_ORI_FREE, to_string(kv_ori_free_));
	// redis_.set(KEY_KV_JOINT_FREE, to_string(kv_joint_free_));

	// // redis_.set(KEY_KP_ORIENTATION_EXP, to_string(kp_ori_exp_));
	// // redis_.set(KEY_KV_ORIENTATION_EXP, to_string(kv_ori_exp_));
	// // redis_.set(KEY_KI_ORIENTATION_EXP, to_string(ki_ori_exp_));
	// // redis_.set(KEY_KP_POSITION_EXP, to_string(kp_pos_exp_));
	// // redis_.set(KEY_MORE_SPEED, to_string(exp_moreSpeed_));
	// // redis_.set(KEY_LESS_DAMPING, to_string(exp_lessDamping_));
	// // redis_.set(KEY_KP_FORCE, to_string(kp_force_));
	// // redis_.set(KEY_KV_FORCE, to_string(kv_force_));
	// // redis_.set(KEY_KI_FORCE, to_string(ki_force_));
	// // redis_.set(KEY_KP_MOMENT, to_string(kp_moment_));
	// // redis_.set(KEY_KV_MOMENT, to_string(kv_moment_));
	// // redis_.set(KEY_KI_MOMENT, to_string(ki_moment_));
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
				try {
					int flag = stoi(redis_.get(KEY_UI_FLAG));
					if (flag) {
						cout << "INIT- Redis synchronized. Switching to joint space controller." << endl;
						controller_state_ = JOINT_SPACE_INITIALIZATION;
					}
				} catch (std::exception& e) {
					cout << e.what() << endl;
				}
				break;

			// Initialize robot to default joint configuration - joint space
			case JOINT_SPACE_INITIALIZATION:
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "ALIGN FREE SPACE- Joint position initialized. Switching to align bottle cap." << endl;
					controller_state_ = DemoProject::ALIGN_FREE_SPACE; //ALIGN_BOTTLE_CAP
				}
				break;
			
			/***********   FREE SPACE   *************/
			case ALIGN_FREE_SPACE:
				if(alignInFreeSpace() == FINISHED){
					cout<< "FREE 2 CONTACT "<< std::endl;
					t_alignment_ = timer_.elapsedTime();
					controller_state_ = DemoProject::FREE_SPACE_TO_CONTACT;
				}
				break;

			case FREE_SPACE_TO_CONTACT:
				switch (freeSpace2Contact()) {
					case FINISHED:
						cout<< "ALIGN BOTTLE CAP" << std::endl;
						controller_state_ = DemoProject::ALIGN_BOTTLE_CAP;
						break;
					case FAILED:
						cout << "FREE SPACE TO CONTACT- Bottle cap not aligned. Switching back to align bottle cap." << endl;
						controller_state_ = ALIGN_FREE_SPACE;
						break;
					default:
						break;
				}
				break;


			case CHECK_FREE_SPACE_ALIGNMENT:
				switch (checkAlignment()) {
					case FINISHED:
						cout << "ALIGN IN CONTACT- Bottle cap aligned. Switching to rewind cap." << endl;
						controller_state_ = ALIGN_BOTTLE_CAP;
						break;
					case FAILED:
						cout << "ALIGN FREE SPACE- Bottle cap not aligned. Switching back to align bottle cap." << endl;
						controller_state_ = ALIGN_FREE_SPACE;
						break;
					default:
						break;
				}
				break;

			/************   CONTACT   **************/
			case ALIGN_BOTTLE_CAP:
				if (alignBottleCapForce() == FINISHED) {  //alignBottleCapExponentialDamping //alignBottleCap //alignBottleCapSimple
					cout << "CHECK ALIGN" << endl;
					controller_state_ = CHECK_ALIGNMENT;
					t_alignment_ = timer_.elapsedTime();
				}
				break;
			
			case CHECK_ALIGNMENT:
				switch (checkAlignment()) {
					case FINISHED:
						cout << "REWIND- Bottle cap aligned. Switching to rewind cap." << endl;
						controller_state_ = REWIND_BOTTLE_CAP;//REWIND_BOTTLE_CAP;//ALIGN_BOTTLE_CAP;
						break;
					case FAILED:
						cout << "ALIGN IN CONTACT- Bottle cap not aligned. Switching back to align bottle cap." << endl;
						controller_state_ = ALIGN_BOTTLE_CAP;
						break;
					default:
						break;
				}
				break;

			case REWIND_BOTTLE_CAP:
				if (rewindBottleCap() == FINISHED) {
					cout << "STABILIZE- Bottle cap rewound. Switching to screw bottle cap." << endl;
					controller_state_ = STABILIZE_REWIND;
					t_alignment_ = timer_.elapsedTime();
				}
				break;

			case STABILIZE_REWIND:
				if (stabilizeRewind() == FINISHED) {
					cout << "SCREW- Success. Switching to screw bottle cap." << endl;
					controller_state_ = SCREW_BOTTLE_CAP;
				}
				break;

			case SCREW_BOTTLE_CAP:
				if (screwBottleCap() == FINISHED) {
					cout << "CHECK- Bottle cap screwed. Switching to check screw." << endl;
					controller_state_ = CHECK_SCREW;
					t_alignment_ = timer_.elapsedTime();
				}
				break;

			case CHECK_SCREW:
				switch (checkScrew()) {
					case FINISHED:
						cout << "FINISHED- Success!!!." << endl;
						break;
					case FAILED:
						cout << "REWIND- Fail. Switching back to rewind bottle cap." << endl;
						controller_state_ = REWIND_BOTTLE_CAP;
						break;
					default:
						break;
				}
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

