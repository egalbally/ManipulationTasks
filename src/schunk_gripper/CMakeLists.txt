#Name the project
project (schunk_gripper_driver)

#Set all the sources required for the library
set (SCHUNK_GRIPPER_DRIVER_SOURCE
	SchunkGripperRedisDriver.cpp
)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable (${PROJECT_NAME} ${CS225A_COMMON_SOURCE} ${SCHUNK_GRIPPER_DRIVER_SOURCE})

# Robotics Library
find_library(RL_LIBRARY NAMES rlhal)

target_link_libraries(${PROJECT_NAME} ${CS225A_COMMON_LIBRARIES} ${RL_LIBRARY})