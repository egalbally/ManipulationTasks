#ifndef SAI2_KUKA_IIWA_H
#define SAI2_KUKA_IIWA_H

#include "redis/RedisClient.h"

#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace KukaIIWA {

/********************
 * Public Constants *
 ********************/

// Default Kuka IIWA server port
const int DEFAULT_PORT = 30200;

// Kuka degrees of freedom
const int DOF = 7;

// Path to the default urdf model and tool file
const char MODEL_FILENAME[] = "resources/kuka_iiwa_driver/kuka_iiwa.urdf";
const char TOOL_FILENAME[]  = "resources/kuka_iiwa_driver/tool.xml";

const std::string KEY_PREFIX = RedisServer::KEY_PREFIX + "kuka_iiwa::";
// Redis keys sent to robot
const std::string KEY_COMMAND_TORQUES         = KEY_PREFIX + "actuators::fgc";
const std::string KEY_DESIRED_JOINT_POSITIONS = KEY_PREFIX + "actuators::q_des";
const std::string KEY_TOOL_MASS               = KEY_PREFIX + "tool::mass"; // Initialized with tool.xml
const std::string KEY_TOOL_COM                = KEY_PREFIX + "tool::com";  // Initialized with tool.xml

// Redis keys returned by robot
const std::string KEY_SENSOR_TORQUES   = KEY_PREFIX + "sensors::torques";
const std::string KEY_JOINT_POSITIONS  = KEY_PREFIX + "sensors::q";
const std::string KEY_JOINT_VELOCITIES = KEY_PREFIX + "sensors::dq";

// Factory function to create VectorXd of size DOF
Eigen::VectorXd VectorXd(double x0, double x1, double x2, double x3, double x4, double x5, double x6) {
	Eigen::VectorXd x(DOF);
	x << x0, x1, x2, x3, x4, x5, x6;
	return x;
}

// Default Kuka home position
const Eigen::VectorXd HOME_POSITION    = VectorXd(90, -30, 0, 60, 0, -90, 0) * M_PI/180.0;
const Eigen::Vector3d HOME_POSITION_EE = Eigen::Vector3d(0, -0.6, 0.6);

// Safety limits
// TODO: Find real velocity and jerk limits
const Eigen::ArrayXd JOINT_LIMITS    = VectorXd(2.9670, 2.0944, 2.9670, 2.0944, 2.9670, 2.0944, 3.0543).array() - 10.0 * M_PI/180.0;

// LBR IIWA 7kg specs: [98.0, 98.0, 100.0, 130.0, 140.0, 180.0, 180.0]
const Eigen::ArrayXd VELOCITY_LIMITS = VectorXd(90.0, 90.0, 95.0, 125.0, 135.0, 170.0, 170.0) * M_PI/180.0;

const Eigen::ArrayXd TORQUE_LIMITS   = VectorXd(176.0, 176.0, 110.0, 110.0, 110.0, 40.0, 40.0); // LBR IIWA 7 R800
// const Eigen::ArrayXd TORQUE_LIMITS   = VectorXd(320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0); // LBR IIWA 14 R820

const Eigen::ArrayXd JERK_LIMITS     = VectorXd(8.8, 8.8, 5.5, 5.5, 5.5, 2.0, 2.0);

// Height limits for the wrist [low high]
const double POS_WRIST_LIMITS[2] = {0.35, 1.05};

}

#endif  // SAI2_KUKA_IIWA_H
