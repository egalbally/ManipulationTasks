#Name the project
project (optoforce_driver)

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

include_directories(${OMD_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${CS225A_COMMON_LIBRARIES} ${OMD_LIBRARIES})
target_link_libraries(optoforce_calibration ${CS225A_COMMON_LIBRARIES} ${OMD_LIBRARIES})