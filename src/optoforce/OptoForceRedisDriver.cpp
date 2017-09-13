/*
 * Driver to use the force sensor optoforce (tested on 6D force sensor only for now)
 * it will create a connection, read the data, transfer the data from count value to real units
 * and publish it in redis with a key named "sai2::optoforceSensor::6Dsensor::force"
 */
#include "OptoForce.h"

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "omd/opto.h"
#include "omd/sensorconfig.h"
#include "omd/optopackage.h"
#include "filters/ButterworthFilter.h"

typedef unsigned long long mytime_t;

sai::ButterworthFilter filter;
const double cutoff_freq = 0.05;  //the cutoff frequency of the filter, in the range of (0 0.5) of sampling freq
bool use_filter = true;

unsigned long long counter = 0;

std::string serialNumber;  // To know which sensitivity report to use
std::string deviceName;  // To know which sensitivity report to use

//std::ofstream force_file;

mytime_t Now()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t millisecs = t.tv_sec * 1000;
    millisecs += t.tv_nsec / (1000 * 1000);
    return millisecs;
}


mytime_t NowMicro()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t microsecs = t.tv_sec * 1000 * 1000;
    microsecs += t.tv_nsec / (1000);
    return microsecs;
}


mytime_t ElapsedTime(mytime_t p_Time)
{
	return Now() - p_Time;
}


mytime_t ElapsedTimeMicro(mytime_t p_Time)
{
	return NowMicro() - p_Time;
}


void MySleep(unsigned long p_uMillisecs)
{
	usleep(p_uMillisecs * 1000);
}

/*
 * Set the config to the DAQ
 * it is a blocking function; returns true, if the sending of
 * configuration is succeeded otherwise it returns false
 */
bool SetConfig(OptoDAQ & p_optoDAQ, int p_iSpeed, int p_iFilter)
{
	SensorConfig sensorConfig;
	sensorConfig.setSpeed(p_iSpeed);
	sensorConfig.setFilter(p_iFilter);
	mytime_t tNow = Now();

	bool bSuccess = false;
	do {
		bSuccess = p_optoDAQ.sendConfig(sensorConfig);
		if (bSuccess) {
			return true;
		}
		if (ElapsedTime(tNow) > 1000) {
			// 1 sec timeout
			return false;
		}
		MySleep(1);
	} while (bSuccess == false);
	return false;
}


void ShowInformation(OptoDAQ & p_optoDAQ, OPort & p_Port)
{
	deviceName = std::string(p_Port.deviceName);
	std::string name = std::string(p_Port.name);
	serialNumber = std::string (p_Port.serialNumber);
	int version = p_optoDAQ.getVersion();
	std::cout<<"Device Name: "<<deviceName<<std::endl;
	std::cout<<"Name: "<<name<<std::endl;
	std::cout<<"Serial Number: "<<serialNumber<<std::endl;
	std::cout<<"Version: "<<version<<std::endl;
}



/*
 * Opens the desired port
 * it returns true if port could be opened otherwise returns false
 */
bool OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex)
{
	MySleep(500); // We wait some ms to be sure about OptoPorts enumerated PortList
	OPort * portList = p_Ports.listPorts(true);
	int iLastSize = p_Ports.getLastSize();
	if (p_iIndex >= iLastSize) {
		// index overflow
		return false;
	}
	bool bSuccess = p_optoDAQ.open(portList[p_iIndex]);
	if (bSuccess) {
		ShowInformation(p_optoDAQ, portList[p_iIndex]);
	}
	return bSuccess;
}


/*
 * Blocking call to read one 3D package (with one second timeout)
 * it return a non-negative number if it succeeded (p_Package will be the read package)
 * otherwise returns -1
 */
int ReadPackage3D(OptoDAQ & p_optoDAQ, OptoPackage & p_Package)
{
	int iSize = -1;
	mytime_t tNow = Now();
	for (;;) {
		iSize = p_optoDAQ.read(p_Package, 0, false);
		if (iSize < 0 || iSize > 0) {
			break;
		}
		// No packages in the queue so we check the timeout
		if (ElapsedTime(tNow) >= 1000) {
			break;
		}
		usleep(100);
//		MySleep(1);
	}
	return iSize;
}


/*
 * Blocking call to read one 6D package (with one second timeout)
 * it return a non-negative number if it succeeded (p_Package will be the read package)
 * otherwise returns -1
 */
int ReadPackage6D(OptoDAQ & p_optoDAQ, OptoPackage6D & p_Package)
{
	int iSize = -1;
	mytime_t tNow = Now();
	for (;;) {
		iSize = p_optoDAQ.read6D(p_Package, false);
		if (iSize < 0 || iSize > 0) {
			break;
		}
		// No packages in the queue so we check the timeout
		if (ElapsedTime(tNow) >= 1000) {
			break;
		}
		usleep(100);
//		MySleep(1);
	}
	return iSize;
}


/*
 * The function determines if the sensor is a 3D sensor
 */
bool Is3DSensor(OptoDAQ & p_optoDAQ)
{
	opto_version optoVersion = p_optoDAQ.getVersion();
	if (optoVersion != _95 && optoVersion != _64) {
		return true;
	}
	return false;
}


/*
 * Remaps the measurements to a right hand basis and uses the sensivity report (in docs)
 * to transform the measurements in N and Nm
 */
void processRaw6DSensorData(const OptoPackage6D& optoPackage, Eigen::VectorXd& data)
{

    double Fz_compression_coeff;
    double Fz_tension_coeff;
    // double Fz_coeff;

    double Fx_coeff;
    double Fy_coeff;

    double Tx_coeff;
    double Ty_coeff;
    double Tz_coeff;

    // square sensor from sensitivity report
    if(deviceName == "95 v1.0")
    {
    	// std::cout << "square sensor detected" << std::endl;
	    double Fz_compression_coeff = 1200./16078.;
	    double Fz_tension_coeff = 150./1925.;
	    // double Fz_coeff = (Fzp_coeff + Fzm_coeff)/2;

	    double Fx_coeff = 150./7098.;
	    double Fy_coeff = 150./8006.;

	    double Tx_coeff = 5./10284.;
	    double Ty_coeff = 5./10134.;
	    double Tz_coeff = 5./14977.;

	    data(0) = optoPackage.Fx * Fx_coeff;
	    data(1) = optoPackage.Fy * Fy_coeff;
		if(optoPackage.Fz < 0) // compression
		{
			data(2) = optoPackage.Fz * Fz_compression_coeff;
		}
		else
		{
			data(2) = optoPackage.Fz * Fz_tension_coeff;
		}
	    data(3) = -optoPackage.Tx * Tx_coeff;
	    data(4) = optoPackage.Ty * Ty_coeff;
	    data(5) = -optoPackage.Tz * Tz_coeff;
    }
    // Round sensor from sensitivity report
    else if(deviceName == "64 v0.9" && serialNumber == "UCE0A076")
    {
    	// std::cout << "round sensor detected" << std::endl;

		double Fz_compression_coeff = 1.0/8.06;
		double Fz_tension_coeff = 1.0/8.06;
		double Fx_coeff = 1.0/44.19;
		double Fy_coeff = 1.0/44.24;

		double Tx_coeff = 1.0/1114.38;
		double Ty_coeff = 1.0/1043.05;
		double Tz_coeff = 1.0/1536.09;

		data(0) = optoPackage.Fx * Fx_coeff;
		data(1) = optoPackage.Fy * Fy_coeff;
		if(optoPackage.Fz < 0) // compression
		{
			data(2) = optoPackage.Fz * Fz_compression_coeff;
		}
		else
		{
			data(2) = optoPackage.Fz * Fz_tension_coeff;
		}
		data(3) = optoPackage.Tx * Tx_coeff;
		data(4) = optoPackage.Ty * Ty_coeff;
		data(5) = optoPackage.Tz * Tz_coeff;

    }



}

void processRaw3DSensorData(const OptoPackage& optoPackage, Eigen::VectorXd& data)
{

    std::cout << "WARNING : processRaw3DSensorData not implemented." << std::endl;

}

void Run3DSensorExample(OptoDAQ & p_optoDAQ)
{
	// start redis client
	RedisClient redis_client;
	redis_client.connect();

    if(use_filter)
    {
        filter.setDimension(3);
        filter.setCutoffFrequency(cutoff_freq);
    }

    Eigen::VectorXd force_raw = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd force_filtered = Eigen::VectorXd::Zero(3);

	mytime_t tNow = Now();
	unsigned int uTotalReadPackages = 0;
	while(true)
	{
		mytime_t tLoopNow = NowMicro();
		OptoPackage optoPackage;
		int iReadSize = ReadPackage3D(p_optoDAQ, optoPackage);
		if (iReadSize < 0) {
			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
			return;
		}
		uTotalReadPackages += (unsigned int)iReadSize;

		// Formatting output in C style
		double dLoopTime = ElapsedTimeMicro(tLoopNow) / 1000.0;
		mytime_t TotalElapsedTime = ElapsedTime(tNow);
		double dTotalTime = (double)TotalElapsedTime / 1000.0; // Elapsed time in sec
		double dFrequency = 0.0;
		if (dTotalTime > 0.0) {
			dFrequency = (uTotalReadPackages / dTotalTime);
		}
//		fprintf(stdout, "Elapsed: %.1f s Loop time: %.2f ms Samples: %u Sample rate: %.2f Hz\r\n", dTotalTime, dLoopTime, uTotalReadPackages, dFrequency);
//		fflush(stdout);

		processRaw3DSensorData(optoPackage, force_raw);

		if(use_filter)
		{
		    force_filtered = filter.update(force_raw);
		}
		else
		{
		    force_filtered = force_raw;
		}

		//send to redis
		redis_client.setEigenMatrix(OptoForce::KEY_3D_SENSOR_FORCE, force_filtered);
	}

}

void Run6DSensorExample(OptoDAQ & p_optoDAQ)
{
	// start redis client
	RedisClient redis_client;
	redis_client.connect();

    if(use_filter)
    {
        filter.setDimension(6);
        filter.setCutoffFrequency(cutoff_freq);
    }

    Eigen::VectorXd force_raw = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd force_filtered = Eigen::VectorXd::Zero(6);

	mytime_t tNow = Now();
	unsigned int uTotalReadPackages = 0;
	int ctr = 0;
//	do {
	while(true)
	{
		mytime_t tLoopNow = NowMicro();
		OptoPackage6D optoPackage;
		int iReadSize = ReadPackage6D(p_optoDAQ, optoPackage);
		if (iReadSize < 0) {
			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
			return;
		}
		uTotalReadPackages += (unsigned int)iReadSize;

		// Formatting output in C style
		double dLoopTime = ElapsedTimeMicro(tLoopNow) / 1000.0;
		mytime_t TotalElapsedTime = ElapsedTime(tNow);
		double dTotalTime = (double)TotalElapsedTime / 1000.0; // Elapsed time in sec
		double dFrequency = 0.0;
		if (dTotalTime > 0.0) {
			dFrequency = (uTotalReadPackages / dTotalTime);
		}
		//fprintf(stdout, "Elapsed: %.1f s Loop time: %.2f ms Samples: %u Sample rate: %.2f Hz\r\n", dTotalTime, dLoopTime, uTotalReadPackages, dFrequency);
		//fflush(stdout);

		processRaw6DSensorData(optoPackage, force_raw);

		if(use_filter)
		{
		    force_filtered = filter.update(force_raw);
		}
		else
		{
		    force_filtered = force_raw;
		}

		// if(counter%500 == 0)
		// {
		// 	std::cout << force_filtered(2) << std::endl;
		// }


		// publish to redis
		redis_client.setEigenMatrix(OptoForce::KEY_6D_SENSOR_FORCE, force_filtered);

		counter++;

//		force_file << force_filtered.transpose() << "\n";

	}


}



int main()
{
	OptoDAQ optoDaq;
	OptoPorts optoPorts;
	// Changeable values, feel free to play with them
	int iPortIndex = 0;  // The index of the port which will be opened

	int iSpeed = 1000; // Speed in Hz (30, 100, 333 or 1000)
	int iFilter = 15;  // Filter cutoff frequency in Hz (15, 50 or 150)
	///////////////////
	if (OpenPort(optoDaq, optoPorts, iPortIndex) == false) {
		std::cout<<"Could not open port"<<std::endl;
		return 0;
	}
	bool bConfig = SetConfig(optoDaq, iSpeed, iFilter);
	if (bConfig == false) {
		std::cout<<"Could not set config"<<std::endl;
		optoDaq.close();
		return 0;
	}

//
//
//	std::cout << "open file\n";
//	force_file.open("forces.txt");


	if (Is3DSensor(optoDaq)) {
		Run3DSensorExample(optoDaq);
	}
	else {
		Run6DSensorExample(optoDaq);
	}


	optoDaq.close();
	return 0;
}

