#ifndef DEMO_PROJECT_H
#define DEMO_PROJECT_H

// CS225a
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "kuka_iiwa/KukaIIWA.h"
#include "optoforce/OptoForce.h"
#include "optitrack/OptiTrack.h"
#include "filters/ButterworthFilter.h"
#include "schunk_gripper/SchunkGripper.h"

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
const std::string KEY_UI_BOTTLE = KukaIIWA::KEY_PREFIX + "ui::bottle_number";

Eigen::Matrix3d Matrix3d(double m11, double m12, double m13,
                         double m21, double m22, double m23,
                         double m31, double m32, double m33) {
	Eigen::Matrix3d M;
	M << m11, m12, m13, m21, m22, m23, m31, m32, m33;
	return M;
}

class DemoProject {

public:

	DemoProject(std::shared_ptr<Model::ModelInterface> robot)
	: robot(robot),
	  controller_state_(REDIS_SYNCHRONIZATION)
	{
		// for (auto& dPhi : vec_dPhi_) {
		// 	dPhi.setZero();
		// }
		R_des_ << 1,  0,  0,
		          0, -1,  0,
		          0,  0, -1;

		// Initialize pivot point filter
		op_point_filter_.setDimension(3);
		op_point_filter_.setCutoffFrequency(0.2);

		// Initialize force sensor filter
		F_sensor_6d_filter_.setDimension(6);
		F_sensor_6d_filter_.setCutoffFrequency(0.01);

		// Initialize optitrack filter
		shelf_filter_.setDimension(7);
		shelf_filter_.setCutoffFrequency(0.01);
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
		GOTO_BOTTLE_CAP,
		GRAB_BOTTLE_CAP,
		GOTO_BOTTLE_VIA_POINT,
		ALIGN_FREE_SPACE,
		FREE_SPACE_TO_CONTACT,
		ALIGN_BOTTLE_CAP,
		REWIND_BOTTLE_CAP,
		REALIGN_BOTTLE_CAP,
		SCREW_BOTTLE_CAP,
		RELEASE_BOTTLE_CAP,
		UNWIND_BOTTLE_CAP
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
	const double kToleranceAlignX = 0.02;
	const double kToleranceAlignDx = 0.001;

	const double kMaxVelocity = 0.1;  // Maximum end effector velocity
	const double kMaxVelocityScrew = 1.5; // 3 radians per second
	const double kMaxVelocityInit = 1; // 1 radian per second

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const int kIntegraldPhiWindow = 2000;
	
	const double kAlignmentWait = 1;
	const double kContactWait = 1;
	const double kGripperWait = 1;
	
	const Eigen::Vector3d kBaseToOptiTrackOffset = Eigen::Vector3d(0.28, -0.69, 0);
	const Eigen::Vector3d kOptiTrackToShelfOffset = Eigen::Vector3d(0, 0, 0.096);
	const Eigen::Vector3d kShelfHeight = Eigen::Vector3d(0, 0, 0.11) - kOptiTrackToShelfOffset;

	const std::vector<Eigen::Vector3d> kContactPositionsInShelf = {
		Eigen::Vector3d(-0.074, -0.005, 0.09),
		Eigen::Vector3d(-0.25,  -0.01,  0.12),
		Eigen::Vector3d(-0.42, 0.005, 0.08),
		Eigen::Vector3d(-0.605,  -0.04,  0.1)
	};

	const std::vector<Eigen::Vector3d> kSafetyDistance2Rim = {
		Eigen::Vector3d(0, 0.02, 0),
		Eigen::Vector3d(0, 0.02, 0),
		Eigen::Vector3d(0, 0.02, 0),
		Eigen::Vector3d(0, 0, 0.02)
	};

	const std::vector<Eigen::Matrix3d> kContactOrientationsInShelf = {
		Matrix3d( 1,  0,    0,
		          0, -0.7071, -0.7071,
		          0,  0.7071, -0.7071),

		Matrix3d( 1,  0,    0,
		          0, -0.7071, -0.7071,
		          0,  0.7071, -0.7071),

		Matrix3d( 1,  0,    0,
		          0, -0.7071, -0.7071,
		          0,  0.7071, -0.7071),

		Matrix3d(1,  0, 0,
		         0, -1, 0,
		         0,  0,-1)
	};

	const std::vector<Eigen::Vector3d> kContactPointsInEE = {
		Eigen::Vector3d(0,-0.035, 0.13),
		Eigen::Vector3d(0,-0.02,  0.13),
		Eigen::Vector3d(0,-0.055, 0.13),
		Eigen::Vector3d(0, 0.035, 0.13)
	};

	// Tool mass
	const double kOptoForceMass = 0.3;
	const std::vector<double> kToolMass={
		0.40, // medium cap
		0.38, // small cap
		0.45, // large cap
		0.40  // medium cap
	};



	const size_t kNumBottles = kContactPositionsInShelf.size();

	// Default gains (used only when keys are nonexistent in Redis)
	std::map<std::string, double> K = {
		{"kp_joint_init", 20},
		{"kv_joint_init",  5},

		{"kp_pos",    15},
		{"kv_pos",     0},
		{"kp_ori",     4},
		{"kv_ori",   0.5},
		{"kp_joint",  15},
		{"kv_joint",   0},
		{"kv_joint_screw", 10},
		{"kp_screw",  15},
		{"kv_screw",   4},

		{"kp_free_to_contact", 0.015},
		{"kp_sliding", 1.5},
		{"kp_bias",      0}, // 1.2

		{"kp_pos_free",   40},
		{"kv_pos_free",   10},
		{"kp_ori_free",   5},
		{"kv_ori_free",   2},
		{"kv_joint_free", 5},

		{"kp_ori_exp",  15},
		{"kv_ori_exp",  20},
		{"ki_ori_exp", 1.5},
		{"kp_pos_exp",  30},
		{"more_speed",   2},
		{"less_damping", 2},

		{"kp_force", 0.25},
		{"kv_force",   3},
		{"ki_force",   1},
		{"kp_moment",  2}, //3 (for small and medium)
		{"kv_moment",  1},
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
	ControllerStatus initializeJointSpace();
	ControllerStatus gotoBottleCap();
	ControllerStatus grabBottleCap();
	ControllerStatus gotoBottleViaPoint();
	ControllerStatus alignInFreeSpace();
	ControllerStatus freeSpace2Contact();
	ControllerStatus alignBottleCap();
	ControllerStatus alignBottleCapExponentialDamping();
	ControllerStatus alignBottleCapSimple();
	ControllerStatus alignBottleCapForce();
	ControllerStatus screwBottleCap();
	ControllerStatus rewindBottleCap();
	ControllerStatus releaseBottleCap();
	ControllerStatus unwindBottleCap();

	Eigen::Vector3d estimatePivotPoint();

	/***** Member variables *****/

	// Robot
	const std::shared_ptr<Model::ModelInterface> robot;

	// Redis
	RedisClient redis_;

	// Timer
	LoopTimer timer_;
	double t_init_ = 0;

	// State machine
	ControllerState controller_state_;

	// Controller variables
	Eigen::VectorXd command_torques_ = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	Eigen::VectorXd q_des_  = KukaIIWA::HOME_POSITION;
	Eigen::VectorXd dq_des_ = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	Eigen::Vector3d x_des_  = KukaIIWA::HOME_POSITION_EE - kPosEndEffector;
	Eigen::Vector3d dx_des_ = Eigen::Vector3d::Zero();
	Eigen::Matrix3d R_des_  = Matrix3d(1, 0, 0,
	                                   0,-1, 0,
	                                   0, 0,-1);
	double gripper_pos_des_ = SchunkGripper::POSITION_MAX;

	Eigen::MatrixXd J_cap_, Jv_, Jw_, Jv_cap_, Jw_cap_;
	Eigen::MatrixXd N_cap_, Nv_, Nv_cap_, Nvw_cap_;
	Eigen::MatrixXd Lambda_cap_, Lambda_x_, Lambda_x_cap_, Lambda_r_cap_;
	Eigen::VectorXd g_;
	Eigen::Vector3d x_, dx_, w_;
	Eigen::Matrix3d R_ee_to_base_;
	Eigen::Vector3d dPhi_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d pos_shelf_;
	Eigen::Quaterniond ori_shelf_;
	ButterworthFilter shelf_filter_;

	Eigen::Vector3d F_sensor_, M_sensor_, F_x_ee_;
	ButterworthFilter F_sensor_6d_filter_;

	// std::vector<Eigen::Vector3d> vec_dPhi_ = std::vector<Eigen::Vector3d>(kIntegraldPhiWindow);
	// int idx_vec_dPhi_ = 0;
	// Eigen::Vector3d integral_dPhi_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d op_point_ = kPosEndEffector;
	ButterworthFilter op_point_filter_;

	bool controller_flag_ = false;
	int idx_bottle_ = 0;
	bool is_screwing_ = false;
	int bottle_flag_ = 0;

	// angle between contact surface normal and cap normal
	// double theta_;
};

#endif  // DEMO_PROJECT_H
