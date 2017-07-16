cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(optoforce_driver)


# CMAKE OPTIONS
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_CXX_FLAGS "-std=c++11")

# BUILD SOURCES AND LIBRARY
set(SOURCES optoforce6DRedisDriver.cpp 
			${PROJECT_SOURCE_DIR}/redis/RedisClient.h
			${PROJECT_SOURCE_DIR}/filters/ButterworthFilter.h
)

# Create a library
add_executable(${PROJECT_NAME} ${SOURCES})

# optoforce
set(OPTOFORCE_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/LINUX_API_V1.5.1/include/)
find_library(OMD_LIBRARIES NAMES OMD PATHS ${CMAKE_CURRENT_LIST_DIR}/LINUX_API_V1.5.1/lib)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - eigen3
find_package(Eigen3 REQUIRED)

# create headers list
set(optoforce_driver_INCLUDE_DIRS   
					${PROJECT_SOURCE_DIR}/redis
					${PROJECT_SOURCE_DIR}/filters
				    ${OPTOFORCE_INCLUDE_DIRS}
				    ${EIGEN3_INCLUDE_DIR}
)

# create libraries list
set(optoforce_driver_LIBRARIES ${OMD_LIBRARIES} -lncurses -lrt -lpthread ${HIREDIS_LIBRARY}
				    )

include_directories( ${optoforce_driver_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${optoforce_driver_LIBRARIES})



