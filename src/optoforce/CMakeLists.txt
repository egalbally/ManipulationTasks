cmake_minimum_required (VERSION 3.1)

#Name the project
project (optoforce_driver)

#Make sure the generated makefile is not shortened
set (CMAKE_VERBOSE_MAKEFILE ON)

cmake_minimum_required(VERSION 3.1)

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
set (OPTOFORCE_DRIVER_SOURCE
	OptoforceRedisDriver.cpp
)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable (${PROJECT_NAME} ${CS225A_COMMON_SOURCE} ${OPTOFORCE_DRIVER_SOURCE})
add_executable (optoforce_calibration ${CS225A_COMMON_SOURCE} OptoforceCalibration.cpp)

# optoforce
set(OMD_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/LINUX_API_V1.5.1/include/)
find_library(OMD_LIBRARIES NAMES OMD PATHS ${CMAKE_CURRENT_LIST_DIR}/LINUX_API_V1.5.1/lib)

# Filters
SET(FILTERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/filters/)

include_directories(${OMD_INCLUDE_DIR} ${FILTERS_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${CS225A_COMMON_LIBRARIES} ${OMD_LIBRARIES})
target_link_libraries(optoforce_calibration ${CS225A_COMMON_LIBRARIES} ${OMD_LIBRARIES})