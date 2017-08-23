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
#include <thread>

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

typedef enum {
	CALIBRATION_MOVING2LOCATION,
	CALIBRATION_SENSING,
	CALIBRATION_MASS_CALCULATION,
	CALIBRATION_HOLD_POSITION,
	CALIBRATION_HOLD_AND_CALCULATE_BIAS,
	CALIBRATION_HOLD_AND_WAIT_FOR_KEY
} CalibrationState;

#define RUN_BOT 1
#define RUN_SIM 0

// Change when running in sim or real robot!!!
bool robotOrSim = RUN_BOT;

// controller global variables
bool runloop = true;
void stop(int){runloop = false;}
unsigned long long controller_counter = 0;
std::string fgc_command_enabled = "";

void sighandler(int sig) //when you press ctrl+C
{ runloop = false; }

bool char_obtained;
void nonblocking_getchar() {
	char c = getchar();
	char_obtained = true;
}

// main loop
int main() {
	
	std::cout << "Loading URDF world model file: " << world_file << std::endl;

	ofstream outputtxt;
	outputtxt.open ("output.txt");

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
	int dof = robot->dof();

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

	// initial desired positions and orientations
	Eigen::Vector3d x_des;
	x_des << 0, -0.6, 0.50;
	Eigen::Matrix3d R_des;
	R_des << 0, -1, 0, -1, 0, 0, 0, 0, -1;
	Eigen::VectorXd q_des = Eigen::VectorXd::Zero(dof); 
	q_des << 90, -30, 0, 60, 0, -90, 0;
	q_des *= M_PI / 180.0;

	// --------------------------------- Calibration variables
	
	// link variable names
	string wrist_link_name = "link5";
	string ee_link_name = "link6";

	// sensed forces and moments
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd ee_sensed_force = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_moment = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_force_inEE = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_force_and_moment = Eigen::VectorXd::Zero(6); 
	
	const int numCalibrationLocations = 3;
	bool CalculatingBiasFlag = 1;

	// initial calibration state
	CalibrationState calibrationState = CALIBRATION_MOVING2LOCATION;

	double posNumber = 0;
	double CAL_POS_TOLERANCE = 0.1;
	double CAL_VEL_TOLERANCE = 0.1;
	double numMeasurements = 0;
	const int maxNumMeasurements = 100;
	
	// desired joint positions and velocities
	Eigen::VectorXd q_calDes = Eigen::VectorXd::Zero(dof); 
	q_calDes << 90, -30, 0, 60, 0, -90, 0; 
	q_calDes *= M_PI / 180.0;
	Eigen::VectorXd qd_calDes = Eigen::VectorXd::Zero(dof); 

	Eigen::Vector3d Fs;
	Eigen::Vector3d Ms;
	Eigen::Vector3d Fbias;
	Eigen::Vector3d Mbias;
	Eigen::VectorXd Fs_x(maxNumMeasurements);   
	Eigen::VectorXd Fs_y(maxNumMeasurements); 
	Eigen::VectorXd Fs_z(maxNumMeasurements); 
	Eigen::VectorXd Ms_x(maxNumMeasurements); 
	Eigen::VectorXd Ms_y(maxNumMeasurements); 
	Eigen::VectorXd Ms_z(maxNumMeasurements);
	Eigen::VectorXd averageMs(3*numCalibrationLocations,1);
	Eigen::Vector3d Ms_0;
	Eigen::Vector3d Ms_1;
	Eigen::Vector3d Ms_2;
	double Fs_0_mod = 0;
	double Fs_1_mod = 0;
	double Fs_2_mod = 0;

	Eigen::Vector3d g_vec_inBase;
	
	Eigen::MatrixXd forceMatrix(3*numCalibrationLocations,3);
	Eigen::Matrix3d forceMatrix0;
	Eigen::Matrix3d forceMatrix1;
	Eigen::Matrix3d forceMatrix2;
	Eigen::MatrixXd forceMatrix_leftInv(3,3*numCalibrationLocations);
	
	Eigen::Vector3d com_pos;
	// ------------------------------------------------------------------
	std::thread io_thread;

	while (runloop) {
		timer.waitForNextLoop();

		// read from Redis
		robot->_q = redis_client.getEigenMatrix(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrix(JOINT_VELOCITIES_KEY);
		if(robotOrSim == RUN_BOT)
			ee_sensed_force_and_moment = redis_client.getEigenMatrix(EE_FORCE_SENSOR_FORCE_KEY); //robot
		else if(robotOrSim == RUN_SIM)
			ee_sensed_force_and_moment.setZero(); //sim

		// force and torque measurements from sensor
		ee_sensed_force = ee_sensed_force_and_moment.head<3>();
		ee_sensed_moment = ee_sensed_force_and_moment.tail<4>();

		// joint angle errors
		Eigen::VectorXd q_calErr = Eigen::VectorXd::Zero(dof);
		Eigen::VectorXd qd_calErr = Eigen::VectorXd::Zero(dof);
		q_calErr = robot->_q - q_calDes;
		qd_calErr = robot->_dq - qd_calDes;
					
		robot->updateModel();
		double time = controller_counter/control_freq;

		// ----------------  Calibration Routine --------------------------

		if (calibrationState == CALIBRATION_MOVING2LOCATION) {
			// Joint space controller w/ velocity saturation	
			qd_calDes = (kp_joint/kv_joint) * -q_calErr;
			double vSat;
			double vMax = 0.5;
			if (qd_calDes.norm() != 0) {
				vSat = vMax/qd_calDes.norm();
			} else {
				vSat = vMax/1e-6;
			}
			if (vSat > 1) {
				vSat = 1;
			} else if (vSat < -1) {
				vSat = -1;
			}		
			
			command_torques = -kv_joint * (robot->_dq - vSat * qd_calDes); 

			// If you have reached a calibration location go to sensing case
			if(q_calErr.norm() < CAL_POS_TOLERANCE && robot->_dq.norm() < CAL_VEL_TOLERANCE)
			{
				cout<< "pos #" << posNumber << " -location reached"<<endl;
				calibrationState = CALIBRATION_SENSING;
			}

		} else if (calibrationState == CALIBRATION_SENSING) {
			// Hold position - setting command torques to zero was experimentally not enough
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 
			
	 		// (1) Save Fs and Ms values in vectors 
			Fs_x(numMeasurements) = ee_sensed_force(0);
			Fs_y(numMeasurements) = ee_sensed_force(1);
			Fs_z(numMeasurements) = ee_sensed_force(2);
			Ms_x(numMeasurements) = ee_sensed_moment(0);
			Ms_y(numMeasurements) = ee_sensed_moment(1);
			Ms_z(numMeasurements) = ee_sensed_moment(2);

			// Switch to calculation state when you have saved the desired num of values 
			numMeasurements++;

			if (numMeasurements == maxNumMeasurements){
				numMeasurements = 0;
				cout<< "pos #" << posNumber << " -finished measurements"<<endl;	
				if (CalculatingBiasFlag){
					calibrationState = CALIBRATION_HOLD_AND_CALCULATE_BIAS;
				}else{
					calibrationState = CALIBRATION_MASS_CALCULATION;
				}	
			}

		} else if (calibrationState == CALIBRATION_MASS_CALCULATION) {
			// Hold position - setting command torques to zero was experimentally not enough
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 

			// (2) Average values of sensed measurements for this position(this will improve precision)
			Fs<< Fs_x.sum()/Fs_x.size(),
				 Fs_y.sum()/Fs_y.size(),
				 Fs_z.sum()/Fs_z.size();

			Ms<< Ms_x.sum()/Ms_x.size(),
				 Ms_y.sum()/Ms_y.size(),
				 Ms_z.sum()/Ms_z.size();
			
			Fs = Fs - Fbias;
			Ms = Ms - Mbias;

			if(posNumber == 0){
		 			Ms_0 = Ms;
		 			Fs_0_mod = Fs.norm();
 			}else if(posNumber ==1){
 					Ms_1 = Ms;
		 			Fs_1_mod = Fs.norm();
	 		}else if(posNumber ==2){
		 			Ms_2 = Ms;
		 			Fs_2_mod = Fs.norm();
	 		}
			cout<< "pos #" << posNumber << " Fs, Ms: " << Fs.transpose() << Ms.transpose() <<endl;

 			// (3) Calculate A (forceMatrix) for the current location
 			//			AX = b where 
 			//				A --> gravity related matrix
 			//				X --> mass and com vector
 			//				b --> Fs, Ms vector
 			
 			//robot->rotation(RotFromBase2EE, ee_link_name);
 			g_vec_inBase << 0.0, 0.0, -9.81;
 			double g = g_vec_inBase.norm();
 			//g_vec_inSensor = RotFromBase2EE.transpose() * g_vec_inBase;
 			//g_vec_inSensor << g_vec_inSensor(0), g_vec_inSensor(1), -g_vec_inSensor(2); //because the force sensor frame is the opposite to that of the EE
			
 			if(posNumber == 0){
		 			forceMatrix0 	<< 0,						Fs(2),					-Fs(1),
		 							   	-Fs(2),					0, 						Fs(0),
		 							   	Fs(1),					-Fs(0),					0;
 			}else if(posNumber ==1){
 					forceMatrix1 	<< 0,						Fs(2),					-Fs(1),
		 							   	-Fs(2),					0, 						Fs(0),
		 							   	Fs(1),					-Fs(0),					0;
	 		}else if(posNumber ==2){
		 			forceMatrix2 	<< 0,						Fs(2),					-Fs(1),
		 							   	-Fs(2),					0, 						Fs(0),
		 							   	Fs(1),					-Fs(0),					0;
		 	}
 			
 			// (4) If you haven't been to all calibration locations -> update desired configuration and move to next location 
 			cout<< "pos #" << posNumber << " -finished calculations"<<endl;
			posNumber++;

 			if (posNumber < numCalibrationLocations )
 			{ 				
 				//update the desired joint angles (IN RADIANS)
 				if (posNumber == 1)
 				 	q_calDes <<  90, -30, 0, 60, -31,-47, 58;
 				// The last position equals the initial one because that's the configuration the controller assumes it's in
				else if (posNumber == 2)
 				  	q_calDes <<  90, -30, 0, 60, 54, -100, -48;

 				q_calDes *= M_PI/180;
 				
 				//Move to next position
 				calibrationState =  CALIBRATION_MOVING2LOCATION;
 			}
 			
 			// (5) If you've visited all calibration locations -> Calculate values, end calibration, and stop robot
 			else
 			{
 				
 				// Hold position - setting command torques to zero was experimentally not enough
				command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 

				// Calculate mass and COM 
				forceMatrix << forceMatrix0, forceMatrix1, forceMatrix2;		
	 			forceMatrix_leftInv = ((forceMatrix.transpose()*forceMatrix).inverse()) * forceMatrix.transpose();
	 			averageMs << Ms_0, Ms_1, Ms_2;

	 			double mass = (Fs_0_mod + Fs_1_mod + Fs_2_mod)/(3*g);
	 			com_pos = -forceMatrix_leftInv * averageMs;

				// end of calibration message
 				cout << " DONE CALIBRATING! =) " << endl;	
				cout << " mass:  " << mass << endl;	
				cout << " distance (sensor to object):  " << com_pos.norm() << endl;	
				cout << " com vector:  " << com_pos << endl;	

				//Move to next position
 				calibrationState =  CALIBRATION_HOLD_POSITION;

 			}

		} else if (calibrationState == CALIBRATION_HOLD_POSITION) {	
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 

		}else if (calibrationState == CALIBRATION_HOLD_AND_CALCULATE_BIAS) {
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 
			cout << "Calculating Bias" <<endl;
				
			Fs<< Fs_x.sum()/Fs_x.size(),
			 	 Fs_y.sum()/Fs_y.size(),
			 	 Fs_z.sum()/Fs_z.size();

			Ms<< Ms_x.sum()/Ms_x.size(),
				 Ms_y.sum()/Ms_y.size(),
				 Ms_z.sum()/Ms_z.size();
			
			Fbias = Fs;
			Mbias = Ms;		

			cout << " Fbias:  " << Fbias << endl;	
			cout << " Mbias:  " << Mbias << endl;	
			cout << "PRESS KEY to continue" <<endl;

			CalculatingBiasFlag = 0;
			calibrationState = CALIBRATION_HOLD_AND_WAIT_FOR_KEY;
			io_thread = std::thread(nonblocking_getchar);

		}else if (calibrationState == CALIBRATION_HOLD_AND_WAIT_FOR_KEY) {
			command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 
			
			if (char_obtained) {
				io_thread.join();
				cout << "Key pressed. Start calibrating." <<endl;
				// getchar();
				calibrationState = CALIBRATION_MOVING2LOCATION;
			}
		}

		redis_client.setEigenMatrix(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;
						
		// Calculate F and M due to the added object after the sensor (Fw, Mw)
		// Note - we will substract these values from the sensed values 
		// 		  to obtain the real force due to contact with the environment
	
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