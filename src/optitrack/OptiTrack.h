#ifndef SAI2_OPTITRACK_H
#define SAI2_OPTITRACK_H

#include "redis/RedisClient.h"

#include <string>

namespace OptiTrack {

/********************
 * Public Constants *
 ********************/

const std::string KEY_PREFIX = RedisServer::KEY_PREFIX + "optitrack::";

const std::string KEY_TIMESTAMP          = KEY_PREFIX + "timestamp";
const std::string KEY_POS_RIGID_BODIES   = KEY_PREFIX + "pos_rigid_bodies";
const std::string KEY_ORI_RIGID_BODIES   = KEY_PREFIX + "ori_rigid_bodies";
const std::string KEY_POS_SINGLE_MARKERS = KEY_PREFIX + "pos_single_markers";
const std::string KEY_ORI_EA_RIGID_BODIES   = KEY_PREFIX + "ori_rigid_bodies_ea";
const std::string KEY_ORI_MAT_RIGID_BODY = KEY_PREFIX + "ori_rigid_body_mat";

}

#endif  // SAI2_OPTITRACK_H