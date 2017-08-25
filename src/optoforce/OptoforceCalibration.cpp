/*
 * Controller used to calculate the mass and COM location
 * of the object attached to the optoforce sensor
 */

/*
 * Note to self. Debugging with gdb:
 *		$ gdb --args ./calibration1
 *		$ (gdb) run
 *		$ bt
 */

// SAI
#include <model/ModelInterface.h>
#include <timer/LoopTimer.h>
#include "redis/RedisClient.h"
#include "kuka_iiwa/KukaIIWA.h"
#include "optoforce/Optoforce.h"

// Std
#include <iostream>
#include <string>
#include <signal.h>

// External
#include <Eigen/Core>

using namespace std;

// link variable names
static const std::string robot_file = "resources/kuka_iiwa_driver/kuka_iiwa.urdf";
static const string ee_link_name = "link6";

// Constants
static const double kPositionEpsilon = 0.1;
static const double kVelocityEpsilon = 0.005;
static const int kNumMeasurements = 500;
static const double kMaxVelocity = 1.0;
static const double kControlFreq = 1000;
static const Eigen::Vector3d g(0, 0, -9.81);

// Transform sensor measurements to ee frame
static const Eigen::Matrix3d R_sensor_to_ee = Eigen::Matrix3d::Identity();

// gains
static const double kp_joint = 15;
static const double kv_joint = 2 * sqrt(kp_joint);

typedef enum {
	CALIBRATION_MOVING2LOCATION,
	CALIBRATION_SENSING,
	CALIBRATION_MASS_CALCULATION,
	CALIBRATION_HOLD_POSITION
} CalibrationState;

static Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d& x) {
	Eigen::Matrix3d X;
	X <<  0   ,  x(2), -x(1),
	     -x(2),  0   ,  x(0),
	      x(1), -x(0),  0   ;
	return X;
}

// controller global variables
static bool g_runloop = true;
static void stop(int) { g_runloop = false; }

// main loop
int main() {
	// Set up signal handler (CTRL+C)
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Load robot
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// Start redis client
	RedisClient redis_client;
	redis_client.connect();

	// Create a loop timer
	LoopTimer timer;
	timer.setLoopFrequency(kControlFreq);  // 1 KHz
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// Initialize model
	robot->_q = redis_client.getEigenMatrix(KukaIIWA::KEY_JOINT_POSITIONS);
	robot->_dq = redis_client.getEigenMatrix(KukaIIWA::KEY_JOINT_VELOCITIES);
	robot->updateModel();

	// --------------------------------- Calibration variables
	// Initial calibration state
	CalibrationState calibrationState = CALIBRATION_MOVING2LOCATION;

	// Vector of desired joint positions
	std::vector<Eigen::VectorXd> vec_q_des;
	vec_q_des.push_back(KukaIIWA::VectorXd(90, -30, 0, 60, 0, -90, 0) * M_PI / 180.0);
	for (int q6 = -45; q6 <= 45; q6 += 45) {
		for (int q7 = -135; q7 <= 135; q7 += 45) {
			vec_q_des.push_back(KukaIIWA::VectorXd(90, -30, 0, 60, 0, q6, q7) * M_PI / 180.0);
		}
	}
	vec_q_des.push_back(KukaIIWA::VectorXd(90, -30, 0, 60, 0, 90, 0) * M_PI / 180.0);
	const int kNumCalibrationLocations = vec_q_des.size();

	// Vector of force sensor readings at each location
	std::vector<Eigen::Vector3d> vec_Fs, vec_Ms, vec_gs;

	// Index variables
	int idx_location = 0;    // Out of kNumCalibrationLocations
	int idx_measurement = 0; // Out of kNumMeasurements

	while (g_runloop) {
		timer.waitForNextLoop();

		// ----------------  Joint space controller  --------------------------
		// Get robot state
		robot->_q = redis_client.getEigenMatrix(KukaIIWA::KEY_JOINT_POSITIONS);
		robot->_dq = redis_client.getEigenMatrix(KukaIIWA::KEY_JOINT_VELOCITIES);

		// Update robot model
		robot->updateModel();

		// Joint space controller w/ velocity saturation
		Eigen::VectorXd q_err = robot->_q - vec_q_des[idx_location];
		Eigen::VectorXd dq_des = -(kp_joint / kv_joint) * q_err;
		double vSat = 1;
		if (dq_des.norm() != 0) {
			vSat = kMaxVelocity / dq_des.norm();
		}
		if (vSat > 1) {
			vSat = 1;
		}
		Eigen::VectorXd command_torques = -kv_joint * (robot->_dq - vSat * dq_des);

		// Send command torques
		redis_client.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, command_torques);

		// ----------------  Force sensor measurements  --------------------------
		// Find gravity in ee frame
		Eigen::Matrix3d R_ee_to_base;
		robot->rotation(R_ee_to_base, ee_link_name);
		Eigen::Vector3d g_ee = R_ee_to_base.transpose() * g;

		// Attempt to get force-torque measurements from sensor
		Eigen::VectorXd FM_sensor = Eigen::VectorXd::Zero(6);
		try {
			FM_sensor = redis_client.getEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE); 
		} catch (...) {
			// No force sensor readings - set 0 for simulation
			FM_sensor.setZero();
		}

		// Convert sensor readings to ee frame
		Eigen::Vector3d F_sensor_ee = R_sensor_to_ee * FM_sensor.head(3);
		Eigen::Vector3d M_sensor_ee = R_sensor_to_ee * FM_sensor.tail(3);

		// ----------------  Calibration Routine --------------------------
		if (calibrationState == CALIBRATION_MOVING2LOCATION) {

			// If you have reached a calibration location go to sensing case
			if (q_err.norm() < kPositionEpsilon && robot->_dq.norm() < kVelocityEpsilon && timer.elapsedTime() > 3.0) {
				cout << "pos #" << idx_location << " -location reached" << endl;
				vec_Fs.push_back(Eigen::Vector3d::Zero());
				vec_Ms.push_back(Eigen::Vector3d::Zero());
				vec_gs.push_back(Eigen::Vector3d::Zero());
				calibrationState = CALIBRATION_SENSING;
			}

		} else if (calibrationState == CALIBRATION_SENSING) {

			// (1) Accumulate Fs and Ms values in appropriate location slots
			vec_Fs[idx_location] += F_sensor_ee;
			vec_Ms[idx_location] += M_sensor_ee;
			vec_gs[idx_location] += g_ee;

			idx_measurement++;

			// Switch to calculation state when you have saved the desired num of values
			if (idx_measurement == kNumMeasurements) {
				idx_measurement = 0;
				cout << "pos #" << idx_location << " -finished measurements" << endl;
				calibrationState = CALIBRATION_MASS_CALCULATION;
			}

		} else if (calibrationState == CALIBRATION_MASS_CALCULATION) {

			// (2) Average values of sensed measurements for this position (this will improve precision)
			vec_Fs.back() /= kNumMeasurements;
			vec_Ms.back() /= kNumMeasurements;
			vec_gs.back() /= kNumMeasurements;

			cout << "pos #" << idx_location << " Fs, Ms, gs: "
			     << vec_Fs[idx_location].transpose() << "; "
			     << vec_Ms[idx_location].transpose() << "; "
			     << vec_gs[idx_location].transpose() << endl;

			idx_location++;

			if (idx_location < kNumCalibrationLocations) {
				// Move to next position
				calibrationState =  CALIBRATION_MOVING2LOCATION;
			} else {
				// (4) If you've visited all calibration locations -> Calculate values, end calibration, and stop robot

				// Calculate mass and Fb
				Eigen::MatrixXd z_I(3*kNumCalibrationLocations, 4);
				Eigen::VectorXd Fs_stack(3*kNumCalibrationLocations);
				for (int i = 0; i < kNumCalibrationLocations; i++) {
					// TODO: Verify norm is 9.81
					z_I.block(3*i,0,3,1) = vec_gs[i] / vec_gs[i].norm();
					z_I.block(3*i,1,3,3) = Eigen::Matrix3d::Identity();
					Fs_stack.segment(3*i,3) = vec_Fs[i];
				}
				Eigen::VectorXd mg_Fb = z_I.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Fs_stack);
				double m = mg_Fb(0) / 9.81;
				Eigen::Vector3d Fb = mg_Fb.tail(3);

				// Calculate r and Mb
				Eigen::MatrixXd zx_I(3*kNumCalibrationLocations, 6);
				Eigen::VectorXd Ms_stack(3*kNumCalibrationLocations);
				for (int i = 0; i < kNumCalibrationLocations; i++) {
					// TODO: Verify norm is 9.81
					zx_I.block(3*i,0,3,3) = crossProductMatrix(-vec_gs[i] / vec_gs[i].norm());
					zx_I.block(3*i,3,3,3) = Eigen::Matrix3d::Identity();
					Ms_stack.segment(3*i,3) = vec_Ms[i];
				}
				Eigen::VectorXd rmg_Mb = zx_I.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Ms_stack);
				Eigen::Vector3d r = rmg_Mb.head(3) / (m * 9.81);
				Eigen::Vector3d Mb = rmg_Mb.tail(3);

				// end of calibration message
				cout << " DONE CALIBRATING! =) " << endl
				     << endl
				     << " Mass:  " << m << endl
				     << " CoM distance (sensor to object):  " << r.norm() << endl
				     << " CoM vector:  " << r.transpose() << endl
				     << " F_bias: " << Fb.transpose() << endl
				     << " M_bias: " << Mb.transpose() << endl
				     << endl
				     << " ||Fs - [z I] * [mg; Fb]||: " << (Fs_stack - z_I * mg_Fb).norm() << endl
				     << " ||Ms - [-zx I] * [rmg; Mb]||: " << (Ms_stack - zx_I * rmg_Mb).norm() << endl
				     << " Fs - [z I] * [mg; Fb]: " << (Fs_stack - z_I * mg_Fb).transpose() << endl
				     << " Ms - [-zx I] * [rmg; Mb]: " << (Ms_stack - zx_I * rmg_Mb).transpose() << endl;

				// Publish values to Redis
				Eigen::VectorXd FM_bias(6);
				FM_bias << Fb, Mb;
				redis_client.setEigenMatrix(Optoforce::KEY_6D_SENSOR_FORCE_BIAS, FM_bias);
				redis_client.set(Optoforce::KEY_6D_SENSOR_MASS, std::to_string(m));
				redis_client.setEigenMatrix(Optoforce::KEY_6D_SENSOR_COM, r);

				// Hold the last position
				calibrationState = CALIBRATION_HOLD_POSITION;
				idx_location = vec_q_des.size() - 1;
			}

		}

	}

	// When you press ctrl+C it will exit the while loop and end up here
	redis_client.setEigenMatrix(KukaIIWA::KEY_COMMAND_TORQUES, Eigen::VectorXd::Zero(KukaIIWA::DOF));

	// Show stats when the experiment is over
	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Loop run time  : " << end_time << " seconds\n";
	std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
	return 0;
}
