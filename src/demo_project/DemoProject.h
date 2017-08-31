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
const std::string KEY_EE_POS             = KukaIIWA::KEY_PREFIX + "tasks::ee_pos";
const std::string KEY_EE_POS_DES         = KukaIIWA::KEY_PREFIX + "tasks::ee_pos_des";
// - read:
const std::string KEY_KP_POSITION        = KukaIIWA::KEY_PREFIX + "tasks::kp_pos";
const std::string KEY_KV_POSITION        = KukaIIWA::KEY_PREFIX + "tasks::kv_pos";
const std::string KEY_KP_ORIENTATION     = KukaIIWA::KEY_PREFIX + "tasks::kp_ori";
const std::string KEY_KV_ORIENTATION     = KukaIIWA::KEY_PREFIX + "tasks::kv_ori";
const std::string KEY_KP_JOINT           = KukaIIWA::KEY_PREFIX + "tasks::kp_joint";
const std::string KEY_KV_JOINT           = KukaIIWA::KEY_PREFIX + "tasks::kv_joint";
const std::string KEY_KP_SCREW           = KukaIIWA::KEY_PREFIX + "tasks::kp_screw";
const std::string KEY_KV_SCREW           = KukaIIWA::KEY_PREFIX + "tasks::kv_screw";
const std::string KEY_KP_JOINT_INIT      = KukaIIWA::KEY_PREFIX + "tasks::kp_joint_init";
const std::string KEY_KV_JOINT_INIT      = KukaIIWA::KEY_PREFIX + "tasks::kv_joint_init";
const std::string KEY_UI_FLAG            = KukaIIWA::KEY_PREFIX + "ui::flag";
const std::string KEY_KP_SLIDING         = KukaIIWA::KEY_PREFIX + "tasks::kp_sliding";
const std::string KEY_KP_BIAS            = KukaIIWA::KEY_PREFIX + "tasks::kp_bias";
const std::string KEY_KP_ORIENTATION_EXP = KukaIIWA::KEY_PREFIX + "tasks::kp_ori_exp";
const std::string KEY_KV_ORIENTATION_EXP = KukaIIWA::KEY_PREFIX + "tasks::kv_ori_exp";
const std::string KEY_KI_ORIENTATION_EXP = KukaIIWA::KEY_PREFIX + "tasks::ki_ori_exp";
const std::string KEY_KP_POSITION_EXP    = KukaIIWA::KEY_PREFIX + "tasks::kp_pos_exp";
const std::string KEY_MORE_SPEED         = KukaIIWA::KEY_PREFIX + "tasks::more_speed";
const std::string KEY_LESS_DAMPING       = KukaIIWA::KEY_PREFIX + "tasks::less_damping";
const std::string THETA                  = KukaIIWA::KEY_PREFIX + "sensor::theta";
const std::string KEY_KP_FORCE           = KukaIIWA::KEY_PREFIX + "tasks::kp_force";
const std::string KEY_KV_FORCE           = KukaIIWA::KEY_PREFIX + "tasks::kv_force";
const std::string KEY_KI_FORCE           = KukaIIWA::KEY_PREFIX + "tasks::ki_force";
const std::string KEY_KP_MOMENT          = KukaIIWA::KEY_PREFIX + "tasks::kp_moment";
const std::string KEY_KV_MOMENT          = KukaIIWA::KEY_PREFIX + "tasks::kv_moment";
const std::string KEY_KI_MOMENT          = KukaIIWA::KEY_PREFIX + "tasks::ki_moment";

class DemoProject {

public:

	DemoProject(std::shared_ptr<Model::ModelInterface> robot)
	: robot(robot),
	  controller_state_(REDIS_SYNCHRONIZATION)
	{
		for (auto& dPhi : vec_dPhi_) {
			dPhi.setZero();
		}

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
		ALIGN_BOTTLE_CAP,
		CHECK_ALIGNMENT,
		REWIND_BOTTLE_CAP,
		STABILIZE_REWIND,
		SCREW_BOTTLE_CAP,
		CHECK_SCREW
	};

	// Return values from computeControlTorques() methods
	enum ControllerStatus {
		RUNNING,  // Not yet converged to goal position
		FINISHED,  // Converged to goal position
		FAILED
	};

	/***** Constants *****/

	const double kToleranceInitQ  = 0.5;  // Joint space initialization tolerance
	const double kToleranceInitDq = 0.1;  // Joint space initialization tolerance
	const double kMaxVelocity = 3;  // Maximum end effector velocity

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const int kIntegraldPhiWindow = 2000;
	
	




	// CHANGE THE OPERATIONAL POINT!!!!!!!!!!!!

	// Medium bottle
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0.035,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,-0.035,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0.035,0,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(-0.035,0,0.12);

	// Small bottle 
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0.02,0.135);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,-0.02,0.135);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0.02,0,0.135);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(-0.02,0,0.135);

	// Large jar
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,0.0575,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0,-0.0575,0.12);
	//const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(0.0575,0,0.12);
	const Eigen::Vector3d kPosEndEffector = Eigen::Vector3d(-0.0575,0,0.12);




	/***** Member functions *****/

	void readRedisValues();
	void updateModel();
	void writeRedisValues();
	ControllerStatus computeJointSpaceControlTorques();
	ControllerStatus computeOperationalSpaceControlTorques();
	ControllerStatus alignBottleCap();
	ControllerStatus alignBottleCapExponentialDamping();
	ControllerStatus alignBottleCapSimple();
	ControllerStatus alignBottleCapForce();
	ControllerStatus checkAlignment();
	ControllerStatus screwBottleCap();
	ControllerStatus rewindBottleCap();
	ControllerStatus stabilizeRewind();
	ControllerStatus checkScrew();

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
	Eigen::Matrix3d R_ee_to_base_;

	std::vector<Eigen::Vector3d> vec_dPhi_ = std::vector<Eigen::Vector3d>(kIntegraldPhiWindow);
	int idx_vec_dPhi_ = 0;
	Eigen::Vector3d integral_dPhi_ = Eigen::Vector3d::Zero();
	double t_alignment_;
	const double kAlignmentWait = 1;
	ButterworthFilter op_point_filter_;
	Eigen::Vector3d op_point_ = kPosEndEffector;

	// Default gains (used only when keys are nonexistent in Redis)
	double kp_pos_ = 15;
	double kv_pos_ = 0;
	double kp_ori_ = 4;
	double kv_ori_ = 0.5;
	double kp_joint_init_ = 10;
	double kv_joint_init_ = 4;
	double kp_joint_ = 15; 
	double kv_joint_ = 0;
	double kp_screw_ = 15;
	double kv_screw_ = 4;
	double kp_sliding_ = 1.5;
	// double kp_bias_ = 1.2; 
	double kp_bias_ = 0.0; 

	// gains for exponential damping during alignment
	double exp_moreSpeed = 2; 
	double exp_lessDamping = 2;
	double kp_ori_exp = 15;
	double kv_ori_exp = 20; 
	double ki_ori_exp = 1.5;
	double kp_pos_exp = 30;	
	double kp_force = 0.5;
	double kv_force = 0;
	double ki_force = 1;
	double kp_moment = 2; //3 (for small and medium)
	double kv_moment = 0;
	double ki_moment = 1;

	// angle between contact surface normal and cap normal
	double theta;
};

#endif  // DEMO_PROJECT_H
