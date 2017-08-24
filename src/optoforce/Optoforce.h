
#include "redis/RedisClient.h"

namespace Optoforce {

const std::string KEY_3D_SENSOR_FORCE = RedisServer::KEY_PREFIX + "optoforce_3d::force";

const std::string KEY_6D_SENSOR_FORCE      = RedisServer::KEY_PREFIX + "optoforce_6d::force";
const std::string KEY_6D_SENSOR_FORCE_BIAS = RedisServer::KEY_PREFIX + "optoforce_6d::force_bias";
const std::string KEY_6D_SENSOR_MASS       = RedisServer::KEY_PREFIX + "optoforce_6d::mass";
const std::string KEY_6D_SENSOR_COM        = RedisServer::KEY_PREFIX + "optoforce_6d::com";
}
