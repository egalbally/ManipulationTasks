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
#include "chai3d.h"
#include "ButterworthFilter.h"
#include "model/ModelInterface.h"
#include "timer/LoopTimer.h"

// For redis publication
#include "redis/RedisClient.h"

using namespace std;
using namespace chai3d;

const std::string world_file = "../resources/01-simulated_force_sensor/world.urdf";
const std::string robot_file = "../../robot_models/kuka_iiwa/01-simulated_force_sensor/kuka_iiwa.urdf";
const std::string robot_name = "Kuka-IIWA";

// Redis keys  
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
const std::string JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
const std::string EE_DESIRED_FORCE_LOGGED_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::desired_force";
const std::string EE_SENSED_FORCE_LOGGED_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::sensed_force";
const std::string JOINT_TORQUES_SENSED_KEY = "sai2::KUKA_IIWA::sensors::torques";
const std::string KP_JOINT_KEY = "sai2::iiwaForceControl::iiwaBot::tasks::kp_joint";
const std::string KV_JOINT_KEY = "sai2::iiwaForceControl::iiwaBot::tasks::kv_joint";

#define CALIBRATION_MOVING2LOCATION 0
#define CALIBRATION_SENSING 1
#define CALIBRATION_MASS_CALCULATION 2
#define CALIBRATION_HOLD_POSITION 3
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

// main loop
int main() {
	
	std::cout << "Loading URDF world model file: " << world_file << std::endl;

	ofstream outputtxt;
	outputtxt.open ("output.txt");

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// set up signal handler (CTRL+C)
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robot TODO: FIND A WAY TO MAKE THIS NOT STUPID :D
	// if(robotOrSim == RUN_BOT)
		auto robot = new Model::ModelInterface(robot_file, Model::rbdl_kuka, Model::urdf, false); // robot
	// if(robotOrSim == RUN_SIM)
	//	 auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false); //sim

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

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
	string redis_buf;
	redis_buf = to_string(kp_joint);
	redis_client.setCommandIs(KP_JOINT_KEY, redis_buf);
	redis_buf = to_string(kv_joint);
	redis_client.setCommandIs(KV_JOINT_KEY, redis_buf);

	//setup lowpass filter
	sai::ButterworthFilter filter;
	filter.setDimension(3);
	filter.setCutoffFrequency(0.05);

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
	Eigen::Vector3d wristOffset;
	wristOffset << 0, 0, 0.1;

	// sensed forces and moments
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd ee_sensed_force = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_moment = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_force_inEE = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_force_and_moment = Eigen::VectorXd::Zero(6); 
	
	static const int numCalibrationLocations = 3;
	static const int NUMBER_CALIBRATION_MEASUREMENTS = 100;

	// initial calibration state
	int calibrationState = CALIBRATION_MOVING2LOCATION;

	double posNumber = 0;
	double CAL_POS_TOLERANCE = 0.1;
	double CAL_VEL_TOLERANCE = 0.1;
	double numMeasurements = 0;
	double maxNumMeasurements = NUMBER_CALIBRATION_MEASUREMENTS;
	
	// desired joint positions and velocities
	Eigen::VectorXd q_calDes = Eigen::VectorXd::Zero(dof); 
	q_calDes << 90, -30, 0, 60, 0, -90, 0; 
	q_calDes *= M_PI / 180.0;
	Eigen::VectorXd qd_calDes = Eigen::VectorXd::Zero(dof); 
	

	Eigen::Vector3d Fs;
	Eigen::Vector3d Ms;
	Eigen::VectorXd Fs_x(maxNumMeasurements);   
	Eigen::VectorXd Fs_y(maxNumMeasurements); 
	Eigen::VectorXd Fs_z(maxNumMeasurements); 
	Eigen::VectorXd Ms_x(maxNumMeasurements); 
	Eigen::VectorXd Ms_y(maxNumMeasurements); 
	Eigen::VectorXd Ms_z(maxNumMeasurements);
	Eigen::VectorXd average_Fs_Ms(6*numCalibrationLocations,1);
	Eigen::VectorXd average_Fs_Ms_0(6,1);
	Eigen::VectorXd average_Fs_Ms_1(6,1);
	Eigen::VectorXd average_Fs_Ms_2(6,1);
	
	Eigen::Matrix3d RotFromBase2EE;

	Eigen::Vector3d g_vec_inBase;
	Eigen::Vector3d g_vec_inSensor;
	
	Eigen::MatrixXd gravityMatrix(6*numCalibrationLocations,4);
	Eigen::MatrixXd gravityMatrix0(6,4);
	Eigen::MatrixXd gravityMatrix1(6,4);
	Eigen::MatrixXd gravityMatrix2(6,4);
	Eigen::MatrixXd gravityMatrix_leftInv(4,6*numCalibrationLocations);
	
	Eigen::VectorXd massAndCOM(4);
	Eigen::Vector3d com_pos;
	Eigen::VectorXd calculatedMassVector(numCalibrationLocations);
	Eigen::VectorXd comPos_x(numCalibrationLocations);
	Eigen::VectorXd comPos_y(numCalibrationLocations);			
	Eigen::VectorXd comPos_z(numCalibrationLocations);

	// ------------------------------------------------------------------
	
	while (runloop) {
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		if(robotOrSim == RUN_BOT)
			redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, ee_sensed_force_and_moment); //robot
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
			
			// cout<< "Fs_x" << Fs_x(numMeasurements) << endl;
			// cout<< "Fs_y" << Fs_y(numMeasurements) << endl;
			// cout<< "Fs_z" << Fs_z(numMeasurements) << endl;
			// cout<< "Ms_x" << Ms_x(numMeasurements) << endl;
			// cout<< "Ms_y" << Ms_y(numMeasurements) << endl;
			// cout<< "Ms_z" << Ms_z(numMeasurements) << endl;

			// Switch to calculation state when you have saved the desired num of values 
			numMeasurements++;
			if (numMeasurements == NUMBER_CALIBRATION_MEASUREMENTS){
				calibrationState = CALIBRATION_MASS_CALCULATION;
				cout<< "pos #" << posNumber << " -finished measurements"<<endl;
				numMeasurements = 0;
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
		
			if(posNumber == 0){
		 			average_Fs_Ms_0 << Fs, Ms;
 			}else if(posNumber ==1){
 					average_Fs_Ms_1 << Fs, Ms;
	 		}else if(posNumber ==2){
		 			average_Fs_Ms_2 << Fs, Ms;
	 		}
			cout<< "pos #" << posNumber << " Fs, Ms: " << Fs.transpose() << Ms.transpose() <<endl;

 			// (3) Calculate A (gravityMatrix) for the current location
 			//			AX = b where 
 			//				A --> gravity related matrix
 			//				X --> mass and com vector
 			//				b --> Fs, Ms vector
 			robot->rotation(RotFromBase2EE, ee_link_name);
 			//cout<< "R " << RotFromBase2EE << endl;
 			g_vec_inBase << 0.0, 0.0, -9.81;
 			g_vec_inSensor = RotFromBase2EE.transpose() * g_vec_inBase;
 			g_vec_inSensor << g_vec_inSensor(0), g_vec_inSensor(1), -g_vec_inSensor(2); //because the force sensor frame is the opposite to that of the EE
			// cout<< "g " << g_vec_inSensor << endl;
			// cout<<"g norm " << g_vec_inSensor.norm()<<endl;

 			if(posNumber == 0){
		 			gravityMatrix0 << g_vec_inSensor(0),		0,						0,						0,
		 							  g_vec_inSensor(1),		0,						0,						0,
		 							  g_vec_inSensor(2),		0,						0,						0,	
		 							  0,						0,						g_vec_inSensor(2),		-g_vec_inSensor(1),
		 							  0,						-g_vec_inSensor(2),		0, 						g_vec_inSensor(0),
		 							  0,						g_vec_inSensor(1),		-g_vec_inSensor(0),		0;
 			}else if(posNumber ==1){
 					gravityMatrix1 << g_vec_inSensor(0),		0,						0,						0,
		 							  g_vec_inSensor(1),		0,						0,						0,
		 							  g_vec_inSensor(2),		0,						0,						0,	
		 							  0,						0,						g_vec_inSensor(2),		-g_vec_inSensor(1),
		 							  0,						-g_vec_inSensor(2),		0, 						g_vec_inSensor(0),
		 							  0,						g_vec_inSensor(1),		-g_vec_inSensor(0),		0;
	 		}else if(posNumber ==2){
		 			gravityMatrix2 << g_vec_inSensor(0),		0,						0,						0,
		 							  g_vec_inSensor(1),		0,						0,						0,
		 							  g_vec_inSensor(2),		0,						0,						0,	
		 							  0,						0,						g_vec_inSensor(2),		-g_vec_inSensor(1),
		 							  0,						-g_vec_inSensor(2),		0, 						g_vec_inSensor(0),
		 							  0,						g_vec_inSensor(1),		-g_vec_inSensor(0),		0;
	 		}
 			
 			// (4) If you haven't been to all calibration locations -> update desired configuration and move to next location 
 			cout<< "pos #" << posNumber << " -finished calculations"<<endl;
			posNumber++;

 			if (posNumber < numCalibrationLocations )
 			{ 				
 				//update the desired joint angles (IN RADIANS)
 				if (posNumber == 1)
 					q_calDes << 155,-34,81,50,-49,-83,46;
 				// The last position equals the initial one because that's the configuration the controller assumes it's in
				else if (posNumber == 2)
 				 	q_calDes << 141,31,48,93,23,-90,40;

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
				gravityMatrix << gravityMatrix0, gravityMatrix1, gravityMatrix2;
				average_Fs_Ms << average_Fs_Ms_0, average_Fs_Ms_1, average_Fs_Ms_2;
	 			
	 			gravityMatrix_leftInv = ((gravityMatrix.transpose()*gravityMatrix).inverse()) * gravityMatrix.transpose();
	 			massAndCOM = gravityMatrix_leftInv * average_Fs_Ms; //X = Ainv * b
	 			double mass = massAndCOM(0);
	 			com_pos << massAndCOM(1)/mass,
	 					   massAndCOM(2)/mass,
	 					   massAndCOM(3)/mass;

				// end of calibration message
 				cout << " DONE CALIBRATING! =) " << endl;	
				cout << " mass:  " << mass << endl;	
				cout << " distance (sensor to object):  " << com_pos.norm() << endl;	
				cout << " com vector:  " << com_pos << endl;	
				cout << " solution vector:  " << massAndCOM << endl;	


				//Move to next position
 				calibrationState =  CALIBRATION_HOLD_POSITION;

 			}

		} else if (calibrationState == CALIBRATION_HOLD_POSITION) {	
				command_torques = robot->_M * (-kp_joint * q_calErr - kv_joint * robot->_dq); 
		}

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;
						
		// Calculate F and M due to the added object after the sensor (Fw, Mw)
		// Note - we will substract these values from the sensed values 
		// 		  to obtain the real force due to contact with the environment
	
	}

	// when you press ctrl+C it will exit the while loop 
	// and come to these lines which stop the robot
	command_torques.setZero();
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // show stats when the experiment is over
    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    return 0;
}