#ifndef DEMO_PROJECT_H
#define DEMO_PROJECT_H

// CS225a
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "kuka_iiwa/KukaIIWA.h"
#include "optoforce/Optoforce.h"
#include "filters/ButterworthFilter.h"

// Standard
#include <string>
#include <thread>

// External
#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <model/ModelInterface.h>

// Redis keys:
// - write:
// - read:
const std::string KEY_UI_FLAG = KukaIIWA::KEY_PREFIX + "ui::flag";

class DemoProject {

public:

	DemoProject(std::shared_ptr<Model::ModelInterface> robot)
	: robot(robot),
	  controller_state_(REDIS_SYNCHRONIZATION)
	{
		for (auto& dPhi : vec_dPhi_) {
			dPhi.setZero();
		}
		R_des_ << 1,  0,  0,
		          0, -1,  0,
		          0,  0, -1;

		// Initialize pivot point filter
		op_point_filter_.setDimension(3);
		op_point_filter_.setCutoffFrequency(0.2);

		// Initialize force sensor filter
		F_sensor_6d_filter_.setDimension(6);
		F_sensor_6d_filter_.setCutoffFrequency(0.01);
	}

	/***** Public functions *****/

	void initialize();
	void runLoop();

protected:

	/***** Enums *****/

	// State enum for controller state machine inside runloop()
	enum ControllerState {
		REDIS_SYNCHRONIZATION,
		JOINT_SPACE_INITIALIZATION,
		ALIGN_FREE_SPACE,
		FREE_SPACE_TO_CONTACT,
		ALIGN_BOTTLE_CAP,
		CHECK_ALIGNMENT,
		REWIND_BOTTLE_CAP,
		STABILIZE_REWIND,
		SCREW_BOTTLE_CAP,
		CHECK_SCREW,
		CHECK_FREE_SPACE_ALIGNMENT
	};

	// Return values from computeControlTorques() methods
	enum ControllerStatus {
		RUNNING,     // Not yet converged to goal position
		STABILIZING, // Waiting to see goal position maintained
		FINISHED,    // Converged to goal position
		FAILED
	};

	/***** Constants *****/

	const double kToleranceInitQ  = 0.5;  // Joint space initialization tolerance
	const double kToleranceInitDq = 0.1;  // Joint space initialization tolerance
	const double kToleranceAlignX = 0.005;
	const double kToleranceAlignDx = 0.001;

	const double kMaxVelocity = 0.1;  // Maximum end effector velocity
	const double kMaxVelocityScrew = 3; // 3 radians per second
	const double kMaxVelocityInit = 1; // 1 radian per second

	const double kSafetyDistance2Rim = 0.03;

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const int kIntegraldPhiWindow = 2000;
	
	const double kAlignmentWait = 1;
	const double kContactWait = 1;

	// Default gains (used only when keys are nonexistent in Redis)
	std::map<std::string, double> K = {
		{"kp_joint_init", 10},
		{"kv_joint_init",  5},

		{"kp_pos",    15},
		{"kv_pos",     0},
		{"kp_ori",     4},
		{"kv_ori",   0.5},
		{"kp_joint",  15},
		{"kv_joint",   0},
		{"kp_screw",  15},
		{"kv_screw",   4},

		{"kp_sliding", 1.5},
		{"kp_bias",      0}, // 1.2

		{"kp_pos_free",   30},
		{"kv_pos_free",   10},
		{"kp_ori_free",   5},
		{"kv_ori_free",   5},
		{"kv_joint_free", 5},

		{"kp_ori_exp",  15},
		{"kv_ori_exp",  20},
		{"ki_ori_exp", 1.5},
		{"kp_pos_exp",  30},
		{"more_speed",   2},
		{"less_damping", 2},

		{"kp_force", 0.5},
		{"kv_force",   0},
		{"ki_force",   1},
		{"kp_moment",  2}, //3 (for small and medium)
		{"kv_moment",  0},
		{"ki_moment",  1}
	};
	




	// CHANGE THE OPERATIONAL POINT!!!!!!!!!!!!

	// Medium bottle
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0.035,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,-0.035,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0.035,0,0.12);
	// const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(-0.035,0,0.12);
	const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0,0.12);

	// Small bottle 
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0.02,0.135);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,-0.02,0.135);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0.02,0,0.135);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(-0.02,0,0.135);

	// Large jar
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0.0575,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,-0.0575,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0.0575,0,0.12);
	// const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(-0.0575,0,0.12);




	/***** Member functions *****/

	void readRedisValues();
	void updateModel();
	void writeRedisValues();
	ControllerStatus computeJointSpaceControlTorques();
	ControllerStatus computeOperationalSpaceControlTorques();
	ControllerStatus alignInFreeSpace();
	ControllerStatus freeSpace2Contact();
	ControllerStatus alignBottleCap();
	ControllerStatus alignBottleCapExponentialDamping();
	ControllerStatus alignBottleCapSimple();
	ControllerStatus alignBottleCapForce();
	ControllerStatus checkAlignment();
	ControllerStatus screwBottleCap();
	ControllerStatus rewindBottleCap();
	ControllerStatus stabilizeRewind();
	ControllerStatus checkScrew();
	ControllerStatus checkFreeSpaceAlignment();

	Eigen::Vector3d estimatePivotPoint();

	/***** Member variables *****/

	// Robot
	const std::shared_ptr<Model::ModelInterface> robot;

	// Redis
	RedisClient redis_;

	// Timer
	LoopTimer timer_;
	double t_curr_;
	uint64_t controller_counter_ = 0;

	// State machine
	ControllerState controller_state_;

	// Controller variables
	Eigen::VectorXd command_torques_ = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	Eigen::VectorXd q_des_           = KukaIIWA::HOME_POSITION;
	Eigen::VectorXd dq_des_          = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	Eigen::Vector3d x_des_           = KukaIIWA::HOME_POSITION_EE - kPosEndEffector;
	Eigen::Vector3d dx_des_          = Eigen::Vector3d::Zero();

	Eigen::MatrixXd J_cap_, Jv_, Jw_, Jv_cap_, Jw_cap_;
	Eigen::MatrixXd N_cap_, Nv_, Nv_cap_, Nvw_cap_;
	Eigen::MatrixXd Lambda_cap_, Lambda_x_, Lambda_x_cap_, Lambda_r_cap_;
	Eigen::VectorXd g_;
	Eigen::Vector3d x_, dx_, w_;
	Eigen::Vector3d F_sensor_, M_sensor_, F_x_ee_;
	ButterworthFilter F_sensor_6d_filter_;
	Eigen::Matrix3d R_ee_to_base_, R_des_;
	Eigen::Vector3d dPhi_ = Eigen::Vector3d::Zero();

	std::vector<Eigen::Vector3d> vec_dPhi_ = std::vector<Eigen::Vector3d>(kIntegraldPhiWindow);
	int idx_vec_dPhi_ = 0;
	Eigen::Vector3d integral_dPhi_ = Eigen::Vector3d::Zero();
	double t_init_;
	ButterworthFilter op_point_filter_;
	Eigen::Vector3d op_point_ = kPosEndEffector;

	bool controller_flag_ = false;

	// angle between contact surface normal and cap normal
	// double theta_;
};

#endif  // DEMO_PROJECT_H
