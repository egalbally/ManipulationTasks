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

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <signal.h>
#include <model/ModelInterface.h>
#include <timer/LoopTimer.h>

// For redis publication
#include "redis/RedisClient.h"
#include "kuka_iiwa/KukaIIWA.h"
#include "optoforce/OptoforceRedisDriver.h"

#include <Eigen/Core>
// #include <thread>

using namespace std;

const std::string world_file = "resources/kuka_iiwa_driver/world.urdf";
const std::string robot_file = "resources/kuka_iiwa_driver/kuka_iiwa.urdf";
const std::string robot_name = "Kuka-IIWA";

// Redis keys
const std::string JOINT_TORQUES_COMMANDED_KEY = KukaIIWA::KEY_COMMAND_TORQUES;
const std::string JOINT_ANGLES_KEY  = KukaIIWA::KEY_JOINT_POSITIONS;
const std::string JOINT_VELOCITIES_KEY = KukaIIWA::KEY_JOINT_VELOCITIES;
const std::string EE_FORCE_SENSOR_FORCE_KEY = Optoforce::KEY_6D_SENSOR_FORCE;
const std::string KP_JOINT_KEY = "sai2::iiwaForceControl::iiwaBot::tasks::kp_joint";
const std::string KV_JOINT_KEY = "sai2::iiwaForceControl::iiwaBot::tasks::kv_joint";

// link variable names
const string wrist_link_name = "link5";
const string ee_link_name = "link6";

// Constants
const double CAL_POS_TOLERANCE = 0.1;
const double CAL_VEL_TOLERANCE = 0.001;
const Eigen::Vector3d g(0, 0, -9.81);
const int maxNumMeasurements = 100;



typedef enum {
	CALIBRATION_MOVING2LOCATION,
	CALIBRATION_SENSING,
	CALIBRATION_MASS_CALCULATION,
	CALIBRATION_HOLD_POSITION
} CalibrationState;

#define RUN_BOT 1
#define RUN_SIM 0

static Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d& x) {
	Eigen::Matrix3d X;
	X <<     0,  x(2), -x(1),
	     -x(2),     0,  x(0),
	      x(1), -x(0),     0;
 	return X;
}

// Change when running in sim or real robot!!!
bool robotOrSim = RUN_BOT;

// controller global variables
bool runloop = true;
void stop(int){runloop = false;}
unsigned long long controller_counter = 0;
std::string fgc_command_enabled = "";

void sighandler(int sig) //when you press ctrl+C
{ runloop = false; }

// main loop
int main() {
	
	std::cout << "Loading URDF world model file: " << world_file << std::endl;

	ofstream outputtxt("output.txt");

	// start redis client
	RedisClient redis_client;
	redis_client.connect();

	// set up signal handler (CTRL+C)
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robot TODO: FIND A WAY TO MAKE THIS NOT STUPID :D
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false); //sim

	// read from Redis
	robot->_q = redis_client.getEigenMatrix(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrix(JOINT_VELOCITIES_KEY);

	// ----------------------------------- Controller setup 
	robot->updateModel();

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);  // 1 KHz
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// gains
	double kp_joint = 15;
	double kv_joint = 2 * sqrt(kp_joint);

	// redis buffer
	redis_client.set(KP_JOINT_KEY, to_string(kp_joint));
	redis_client.set(KV_JOINT_KEY, to_string(kv_joint));

	// Transform sensor measurements to EE frame
	Eigen::Matrix3d R_sensor_to_EE;
	R_sensor_to_EE << -1/sqrt(2), -1/sqrt(2), 	0,
		               1/sqrt(2), -1/sqrt(2), 	0,
		               0, 		   0, 		  	-1;

	// --------------------------------- Calibration variables
	

	// command torques
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(KukaIIWA::DOF);

	// initial calibration state
	CalibrationState calibrationState = CALIBRATION_MOVING2LOCATION;

	int posNumber = 0;
	int numMeasurements = 0;
	
	// desired joint positions and velocities
	std::vector<Eigen::VectorXd> vec_q_des;
	Eigen::VectorXd q_temp(KukaIIWA::DOF); 
	q_temp << 90, -30, 0, 60, 0, -90, 0;
	q_temp *= M_PI / 180.0;
	vec_q_des.push_back(q_temp);
	for (int q6 = -45; q6 <= 0; q6 += 45) {
		for (int q7 = -135; q7 <= 135; q7 += 45) {
			q_temp << 90, -30, 0, 60, 0, q6, q7;
			q_temp *= M_PI / 180.0;
			vec_q_des.push_back(q_temp);
		}
	}
	Eigen::VectorXd dq_des = Eigen::VectorXd::Zero(KukaIIWA::DOF);
	std::vector<Eigen::Vector3d> vec_Fs;
	std::vector<Eigen::Vector3d> vec_Ms;
	std::vector<Eigen::Vector3d> vec_gs;
	const int numCalibrationLocations = vec_q_des.size();

	// Force sensor readings at a given position
	Eigen::MatrixXd Fs_i(maxNumMeasurements, 3);
	Eigen::MatrixXd Ms_i(maxNumMeasurements, 3);
	Eigen::MatrixXd gs_i(maxNumMeasurements, 3);

	while (runloop) {
		timer.waitForNextLoop();

		// read from Redis
		robot->_q = redis_client.getEigenMatrix(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrix(JOINT_VELOCITIES_KEY);

		Eigen::VectorXd ee_sensed_force_and_moment = Eigen::VectorXd::Zero(6); 
		if(robotOrSim == RUN_BOT)
			ee_sensed_force_and_moment = redis_client.getEigenMatrix(EE_FORCE_SENSOR_FORCE_KEY); //robot
		else if(robotOrSim == RUN_SIM)
			ee_sensed_force_and_moment.setZero(); //sim

		// force and torque measurements from sensor
		Eigen::Vector3d ee_sensed_force = R_sensor_to_EE * ee_sensed_force_and_moment.head(3);
		Eigen::Vector3d ee_sensed_moment = R_sensor_to_EE * ee_sensed_force_and_moment.tail(3);

		// joint angle errors
		Eigen::VectorXd q_calErr = robot->_q - vec_q_des[posNumber];
		Eigen::VectorXd qd_calErr = robot->_dq - dq_des;
					
		robot->updateModel();
		double time = controller_counter/control_freq;

		// get gravity
		Eigen::Matrix3d Rot_EE_to_base;
		robot->rotation(Rot_EE_to_base, ee_link_name);
		Eigen::Vector3d g_ee = Rot_EE_to_base.transpose() * g;

		// ----------------  Calibration Routine --------------------------

		if (calibrationState == CALIBRATION_MOVING2LOCATION) {
			// Joint space controller w/ velocity saturation	
			dq_des = (kp_joint/kv_joint) * -q_calErr;
			double vSat;
			double vMax = 0.5;
			if (dq_des.norm() != 0) {
				vSat = vMax/dq_des.norm();
			} else {
				vSat = vMax/1e-6;
			}
			if (vSat > 1) {
				vSat = 1;
			} else if (vSat < -1) {
				vSat = -1;
			}		
			
			command_torques = -kv_joint * (robot->_dq - vSat * dq_des); 

			// If you have reached a calibration location go to sensing case
			if(q_calErr.norm() < CAL_POS_TOLERANCE && robot->_dq.norm() < CAL_VEL_TOLERANCE && timer.elapsedTime() > 3.0)
			{
				cout<< "pos #" << posNumber << " -location reached"<<endl;
				calibrationState = CALIBRATION_SENSING;
			}

		} else if (calibrationState == CALIBRATION_SENSING) {
			// Hold position - setting command torques to zero was experimentally not enough
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 
			
	 		// (1) Save Fs and Ms values in vectors 
	 		Fs_i.row(numMeasurements) = ee_sensed_force;
	 		Ms_i.row(numMeasurements) = ee_sensed_moment;
	 		gs_i.row(numMeasurements) = g_ee;

			// Switch to calculation state when you have saved the desired num of values 
			numMeasurements++;

			if (numMeasurements == maxNumMeasurements){
				numMeasurements = 0;
				cout<< "pos #" << posNumber << " -finished measurements"<<endl;	
				calibrationState = CALIBRATION_MASS_CALCULATION;
			}

		} else if (calibrationState == CALIBRATION_MASS_CALCULATION) {
			// Hold position - setting command torques to zero was experimentally not enough
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 

			// (2) Average values of sensed measurements for this position(this will improve precision)
			vec_Fs.push_back(Fs_i.colwise().mean());
			vec_Ms.push_back(Ms_i.colwise().mean());
			vec_gs.push_back(gs_i.colwise().mean());

			cout << "pos #" << posNumber << " Fs, Ms, gs: " << vec_Fs[vec_Fs.size()-1].transpose() << "; " << vec_Ms[vec_Ms.size()-1].transpose() << "; " << vec_gs[vec_gs.size()-1].transpose() << endl;
 			
 			// (4) If you haven't been to all calibration locations -> update desired configuration and move to next location 
 			cout<< "pos #" << posNumber << " -finished calculations"<<endl;
			posNumber++;

 			if (posNumber < numCalibrationLocations )
 			{ 				
 				//Move to next position
 				calibrationState =  CALIBRATION_MOVING2LOCATION;
 			}
 			
 			// (5) If you've visited all calibration locations -> Calculate values, end calibration, and stop robot
 			else
 			{
 				// Calculate mass and Fbias
 				Eigen::MatrixXd gI(3*numCalibrationLocations, 4);
 				Eigen::VectorXd Fs_stack(3*numCalibrationLocations);
 				for (int i = 0; i < numCalibrationLocations; i++) {
 					gI.block(3*i,0,3,1) = vec_gs[i];
 					gI.block(3*i,1,3,3) = Eigen::Matrix3d::Identity();
 					Fs_stack.segment(3*i,3) = vec_Fs[i];
 				}
 				Eigen::VectorXd m_Fbias = gI.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Fs_stack);
 				double m = m_Fbias(0);
 				Eigen::Vector3d Fbias = m_Fbias.tail(3);

 				// Calculate r and Mbias
 				Eigen::MatrixXd mgxI(3*numCalibrationLocations, 6);
 				Eigen::VectorXd Ms_stack(3*numCalibrationLocations);
 				for (int i = 0; i < numCalibrationLocations; i++) {
 					mgxI.block(3*i,0,3,3) = crossProductMatrix(-m*vec_gs[i]);
 					mgxI.block(3*i,3,3,3) = Eigen::Matrix3d::Identity();
 					Ms_stack.segment(3*i,3) = vec_Ms[i];
 				}
 				Eigen::VectorXd r_Mbias = mgxI.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Ms_stack);
 				Eigen::Vector3d r = r_Mbias.head(3);
 				Eigen::Vector3d Mbias = r_Mbias.tail(3);
 				
 				// Hold position - setting command torques to zero was experimentally not enough
				command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 

				// end of calibration message
 				cout << " DONE CALIBRATING! =) " << endl;	
				cout << " mass:  " << m << endl;	
				cout << " distance (sensor to object):  " << r.norm() << endl;	
				cout << " com vector:  " << r.transpose() << endl;
				cout << " F_bias: " << Fbias.transpose() << endl;
				cout << " M_bias: " << Mbias.transpose() << endl;

				//Move to next position
 				calibrationState = CALIBRATION_HOLD_POSITION;
 				cout << "Going to CALIBRATION_HOLD_POSITION" << endl;

 				// Hold the last position
 				posNumber--;
 			}

		} else if (calibrationState == CALIBRATION_HOLD_POSITION) {
			cout << command_torques.transpose() << endl;
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 
		}

		redis_client.setEigenMatrix(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;
	
	}

	// when you press ctrl+C it will exit the while loop 
	// and come to these lines which stop the robot
	command_torques.setZero();
    redis_client.setEigenMatrix(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // show stats when the experiment is over
    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    return 0;
}