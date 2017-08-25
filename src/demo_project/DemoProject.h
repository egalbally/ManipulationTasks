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

class DemoProject {

public:

	DemoProject(std::shared_ptr<Model::ModelInterface> robot,
		        const std::string &robot_name) :
		robot(robot),
		dof(robot->dof()),
		KEY_COMMAND_TORQUES (kRedisKeyPrefix + robot_name + "::actuators::fgc"),
		KEY_EE_POS          (kRedisKeyPrefix + robot_name + "::tasks::ee_pos"),
		KEY_EE_POS_DES      (kRedisKeyPrefix + robot_name + "::tasks::ee_pos_des"),
		KEY_JOINT_POSITIONS (kRedisKeyPrefix + robot_name + "::sensors::q"),
		KEY_JOINT_VELOCITIES(kRedisKeyPrefix + robot_name + "::sensors::dq"),
	    THETA(kRedisKeyPrefix + robot_name + "::sensor::theta"),
		KEY_TIMESTAMP       (kRedisKeyPrefix + robot_name + "::timestamp"),
		KEY_KP_POSITION     (kRedisKeyPrefix + robot_name + "::tasks::kp_pos"),
		KEY_KV_POSITION     (kRedisKeyPrefix + robot_name + "::tasks::kv_pos"),
		KEY_KP_ORIENTATION  (kRedisKeyPrefix + robot_name + "::tasks::kp_ori"),
		KEY_KV_ORIENTATION  (kRedisKeyPrefix + robot_name + "::tasks::kv_ori"),
		KEY_KP_JOINT        (kRedisKeyPrefix + robot_name + "::tasks::kp_joint"),
		KEY_KV_JOINT        (kRedisKeyPrefix + robot_name + "::tasks::kv_joint"),
		KEY_KP_JOINT_INIT   (kRedisKeyPrefix + robot_name + "::tasks::kp_joint_init"),
		KEY_KV_JOINT_INIT   (kRedisKeyPrefix + robot_name + "::tasks::kv_joint_init"),
		KEY_KP_SCREW        (kRedisKeyPrefix + robot_name + "::tasks::kp_screw"),
		KEY_KV_SCREW        (kRedisKeyPrefix + robot_name + "::tasks::kv_screw"),
		KEY_KP_SLIDING      (kRedisKeyPrefix + robot_name + "::tasks::kp_sliding"),
		KEY_KP_BIAS         (kRedisKeyPrefix + robot_name + "::tasks::kp_bias"),
		KEY_UI_FLAG         (kRedisKeyPrefix + robot_name + "::ui::flag"),
		KEY_KP_ORIENTATION_EXP(kRedisKeyPrefix + robot_name + "::tasks::kp_ori_exp"),
	    KEY_KV_ORIENTATION_EXP(kRedisKeyPrefix + robot_name + "::tasks::kv_ori_exp"),
	    KEY_KI_ORIENTATION_EXP(kRedisKeyPrefix + robot_name + "::tasks::ki_ori_exp"),
	    KEY_KP_POSITION_EXP (kRedisKeyPrefix + robot_name + "::tasks::kp_pos_exp"),
	    KEY_MORE_SPEED(kRedisKeyPrefix + robot_name + "::tasks::more_speed"),
	    KEY_LESS_DAMPING(kRedisKeyPrefix + robot_name + "::tasks::less_damping"),

		command_torques_(dof),
		J_cap_(6, dof),
		Jv_(3, dof),
		Jw_(3, dof),
		Jv_cap_(3, dof),
		Jw_cap_(3, dof),
		N_cap_(dof, dof),
		Nv_(dof, dof),
		Nv_cap_(dof, dof),
		Nvw_cap_(dof, dof),
		Lambda_cap_(6, 6),
		Lambda_x_(3, 3),
		Lambda_x_cap_(3, 3),
		Lambda_r_cap_(3, 3),
		g_(dof),
		q_des_(dof),
		dq_des_(dof),
		controller_state_(REDIS_SYNCHRONIZATION)
	{
		command_torques_.setZero();

		// Home configuration for Kuka iiwa
		// q_des_ << 90, -30, 0, 60, 0, -90, 0;
		q_des_ << 90, -30, 0, 60, 0, -90, -90;
		q_des_ *= M_PI / 180.0;
		dq_des_.setZero();

		// Desired end effector position
		x_des_ = KukaIIWA::HOME_POSITION_EE - Eigen::Vector3d(0,0,0.11);
		dx_des_.setZero();

		for (auto& dPhi : vec_dPhi_) {
			dPhi.setZero();
		}

		// Initialize pivot point filter
		op_point_filter_.setDimension(3);
		op_point_filter_.setCutoffFrequency(0.2);
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
		SCREW_BOTTLE_CAP
	};

	// Return values from computeControlTorques() methods
	enum ControllerStatus {
		RUNNING,  // Not yet converged to goal position
		FINISHED,  // Converged to goal position
		FAILED
	};

	/***** Constants *****/

	const int dof;  // Initialized with robot model
	const double kToleranceInitQ  = 0.5;  // Joint space initialization tolerance
	const double kToleranceInitDq = 0.1;  // Joint space initialization tolerance
	const double kMaxVelocity = 3;  // Maximum end effector velocity

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const int kIntegraldPhiWindow = 2000;

	const std::string kRedisHostname = "127.0.0.1";
	const int kRedisPort = 6379;

	// Redis keys:
	const std::string kRedisKeyPrefix = RedisServer::KEY_PREFIX;
	// - write:
	const std::string KEY_COMMAND_TORQUES;
	const std::string KEY_EE_POS;
	const std::string KEY_EE_POS_DES;
	// - read:
	const std::string KEY_JOINT_POSITIONS;
	const std::string KEY_JOINT_VELOCITIES;
	const std::string KEY_TIMESTAMP;
	const std::string KEY_KP_POSITION;
	const std::string KEY_KV_POSITION;
	const std::string KEY_KP_ORIENTATION;
	const std::string KEY_KV_ORIENTATION;
	const std::string KEY_KP_JOINT;
	const std::string KEY_KV_JOINT;
	const std::string KEY_KP_SCREW;
	const std::string KEY_KV_SCREW;
	const std::string KEY_KP_JOINT_INIT;
	const std::string KEY_KV_JOINT_INIT;
	const std::string KEY_UI_FLAG;
	const std::string KEY_KP_SLIDING;
	const std::string KEY_KP_BIAS;
	const std::string KEY_KP_ORIENTATION_EXP;
	const std::string KEY_KV_ORIENTATION_EXP;
	const std::string KEY_KI_ORIENTATION_EXP;
	const std::string KEY_KP_POSITION_EXP;
	const std::string KEY_MORE_SPEED;
	const std::string KEY_LESS_DAMPING;
	const std::string THETA;

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
	Eigen::VectorXd command_torques_;
	Eigen::MatrixXd J_cap_, Jv_, Jw_, Jv_cap_, Jw_cap_;
	Eigen::MatrixXd N_cap_, Nv_, Nv_cap_, Nvw_cap_;
	Eigen::MatrixXd Lambda_cap_, Lambda_x_, Lambda_x_cap_, Lambda_r_cap_;
	Eigen::VectorXd g_;
	Eigen::Vector3d x_, dx_, w_;
	Eigen::VectorXd q_des_, dq_des_;
	Eigen::Vector3d x_des_, dx_des_;
	Eigen::Vector3d F_sensor_, M_sensor_;
	Eigen::Matrix3d R_ee_to_base_;
	Eigen::Matrix3d R_sensor_to_ee_;
	std::vector<Eigen::Vector3d> vec_dPhi_ = std::vector<Eigen::Vector3d>(kIntegraldPhiWindow);
	int idx_vec_dPhi_ = 0;
	Eigen::Vector3d integral_dPhi_ = Eigen::Vector3d::Zero();
	double t_alignment_;
	const double kAlignmentWait = 1;
	ButterworthFilter op_point_filter_;
	Eigen::Vector3d op_point_;

	// Default gains (used only when keys are nonexistent in Redis)
	double kp_pos_ = 30;
	//double kp_pos_ = 40;
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
	double exp_moreSpeed = 2; //.8
	double exp_lessDamping = 2;
	double kp_ori_exp = 15;
	double kv_ori_exp = 20; //10
	double ki_ori_exp = 1.5;
	double kp_pos_exp = 30;	//15

	// angle between contact surface normal and cap normal
	double theta;
};

#endif  // DEMO_PROJECT_H
