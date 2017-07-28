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

// Path to the urdf model and tool file
const char MODEL_FILENAME[] = "resources/kuka_iiwa_driver/kuka_iiwa.urdf";
const char TOOL_FILENAME[]  = "resources/kuka_iiwa_driver/tool.xml";

const std::string KEY_PREFIX = RedisServer::KEY_PREFIX + "kuka_iiwa::";
// Redis keys sent to robot
const std::string KEY_COMMAND_TORQUES         = KEY_PREFIX + "actuators::fgc";
const std::string KEY_DESIRED_JOINT_POSITIONS = KEY_PREFIX + "actuators::q_des";

// Redis keys returned by robot
const std::string KEY_SENSOR_TORQUES   = KEY_PREFIX + "sensors::torques";
const std::string KEY_JOINT_POSITIONS  = KEY_PREFIX + "sensors::q";
const std::string KEY_JOINT_VELOCITIES = KEY_PREFIX + "sensors::dq";

// Default Kuka home position
const double _ARR_HOME_POSITION[KukaIIWA::DOF]   = {90, -30, 0, 60, 0, -90, 0};
const Eigen::VectorXd HOME_POSITION = Eigen::Map<const Eigen::VectorXd>(_ARR_HOME_POSITION, KukaIIWA::DOF) * M_PI/180.0;

// Safety limits (for initialization purposes only - use Eigen mappings instead)
// TODO: Find real velocity and jerk limits
const double _ARR_JOINT_LIMITS[KukaIIWA::DOF]    = {2.9670, 2.0944, 2.9670, 2.0944, 2.9670, 2.0944, 3.0543};

const double _ARR_VELOCITY_LIMITS[KukaIIWA::DOF] = {90.0, 90.0, 95.0, 125.0, 135.0, 170.0, 170.0};
// const double ARR_VELOCITY_LIMITS[KukaIIWA::DOF] = {98.0, 98.0, 100.0, 130.0, 140.0, 180.0, 180.0}; // for LBR IIWA 7kg from specs

const double _ARR_TORQUE_LIMITS[KukaIIWA::DOF]   = {176.0, 176.0, 110.0, 110.0, 110.0, 40.0, 40.0}; // LBR iiwa 7 R800
// const double _ARR_TORQUE_LIMITS[KukaIIWA::DOF]   = {320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0}; // LBR iiwa 14 R820

const double _ARR_JERK_LIMITS[KukaIIWA::DOF]     = {8.8, 8.8, 5.5, 5.5, 5.5, 2.0, 2.0};

// Eigen mappings to safety limits
const Eigen::ArrayXd JOINT_LIMITS    = Eigen::Array<double,DOF,1>(_ARR_JOINT_LIMITS) - 10.0 * M_PI/180.0;
const Eigen::ArrayXd TORQUE_LIMITS   = Eigen::Array<double,DOF,1>(_ARR_TORQUE_LIMITS);
const Eigen::ArrayXd VELOCITY_LIMITS = Eigen::Array<double,DOF,1>(_ARR_VELOCITY_LIMITS) * M_PI/180.0;
const Eigen::ArrayXd JERK_LIMITS     = Eigen::Array<double,DOF,1>(_ARR_JERK_LIMITS);

// Height limits for the wrist [low high]
const double POS_WRIST_LIMITS[2] = {0.45, 1.05};

}

#endif  // SAI2_KUKA_IIWA_H
