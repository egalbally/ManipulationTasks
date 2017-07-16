cmake_minimum_required (VERSION 3.1)

#Name the project
project (kuka_iiwa_driver)

#Make sure the generated makefile is not shortened
set (CMAKE_VERBOSE_MAKEFILE ON)

cmake_minimum_required(VERSION 3.1)

option(USE_KUKA_LBR_DYNAMICS USE_KUKA_LBR_DYNAMICS ON)


################Initialize the executable#################

#Set the compilation flags
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

set (CS225A_COMMON_SOURCE
	${PROJECT_SOURCE_DIR}/../redis/RedisClient.cpp
	${PROJECT_SOURCE_DIR}/../timer/LoopTimer.cpp
)
include_directories (${PROJECT_SOURCE_DIR}/..)

#Set all the sources required for the library
set (KUKA_DRIVER_SOURCE
	RedisDriver.cpp
)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable (${PROJECT_NAME} ${CS225A_COMMON_SOURCE} ${KUKA_DRIVER_SOURCE})

# KUKA IIWA FRI SDK
message(${CMAKE_CURRENT_SOURCE_DIR})
set(FRI_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/FRI-Client-SDK)
set(FRI_INCLUDE_DIR ${FRI_BASE_DIR}/include)
find_library(FRI_LIBRARIES FRIClient PATHS ${FRI_BASE_DIR}/lib)
# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O2 -Wall -Wextra -pedantic -std=c++11 -fPIC -fopenmp")
# add_definitions(-DHAVE_SOCKLEN_T -DPB_SYSTEM_HEADER=\"pb_syshdr.h\" -DHAVE_STDINT_H -DHAVE_STDDEF_H -DHAVE_STDBOOL_H -DHAVE_STDLIB_H -DHAVE_STRING_H)

# KUKA LBR Dynamics
if (USE_KUKA_LBR_DYNAMICS)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/KukaLBRDynamics)
    option(USE_KUKA_RBDL    "USE_KUKA_RBDL"     OFF)
    find_package (KukaLBRDynamics REQUIRED)
    include_directories(${KukaLBRDynamics_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${KukaLBRDynamics_LIBRARIES})
    add_definitions(-DUSE_KUKA_LBR_DYNAMICS)

    FIND_LIBRARY(TINYXML_LIBRARIES tinyxml2)
    target_link_libraries(${PROJECT_NAME} ${TINYXML_LIBRARIES})
endif(USE_KUKA_LBR_DYNAMICS)

# Filters
SET(FILTERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/filters/)

include_directories(${FRI_INCLUDE_DIR} ${FILTERS_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${CS225A_COMMON_LIBRARIES} ${FRI_LIBRARIES})

# Hold pos demo controller
add_executable(kuka_hold_pos ${CS225A_COMMON_SOURCE} kuka_hold_pos_main.cpp)
target_link_libraries(kuka_hold_pos ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/kuka_iiwa_driver)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY tool.xml DESTINATION ${APP_RESOURCE_DIR})

SET(KUKA_HOLD_POS_RESOURCE_DIR ${RESOURCE_DIR}/kuka_hold_pos)
FILE(MAKE_DIRECTORY ${KUKA_HOLD_POS_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf DESTINATION ${KUKA_HOLD_POS_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${KUKA_HOLD_POS_RESOURCE_DIR})
