cmake_minimum_required(VERSION 3.1)

project(CS225A)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Explicitly include for Macs without XCode initialized
include_directories(/usr/include /usr/local/include)

# Set common source
set(CS225A_COMMON_SOURCE
	${PROJECT_SOURCE_DIR}/src/redis/RedisClient.cpp
	${PROJECT_SOURCE_DIR}/src/timer/LoopTimer.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/src)

# Set common dependencies

# - SAI2-SIMULATION. TODO: this should not be needed. something fishy.
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-MODEL
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - SAI2-COMMON
#find_package(SAI2-COMMON REQUIRED)
#include_directories(${SAI2-COMMON_INCLUDE_DIRS})

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - jsoncpp
if(CMAKE_SYSTEM_NAME MATCHES Darwin) 
	find_library(JSONCPP_LIBRARY jsoncpp)
elseif(CMAKE_SYSTEM_NAME MATCHES Linux)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(JSONCPP jsoncpp)
	include_directories(${JSONCPP_INCLUDE_DIRS})
	link_directories(${JSONCPP_LIBRARY_DIRS})
	set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif()

set(CS225A_COMMON_LIBRARIES
	${SAI2-COMMON_LIBRARIES}
	${HIREDIS_LIBRARY}
	${JSONCPP_LIBRARY}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
	${SAI2-MODEL_LIBRARIES}
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
)

# Add apps
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin)
set(CS225A_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(src/visualization)
add_subdirectory(src/simulation)
# add_subdirectory(src/hw0)
# add_subdirectory(src/hw1)
# add_subdirectory(src/hw2)
# add_subdirectory(src/hw3)
add_subdirectory(src/demo_project)

#add_subdirectory(src/kuka_iiwa)
add_subdirectory(src/optoforce)
add_subdirectory(src/schunk_gripper)
add_subdirectory(src/kuka_iiwa)

