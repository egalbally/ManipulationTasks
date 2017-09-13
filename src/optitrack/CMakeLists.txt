# Find natnetlinux
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
include_directories(${PROJECT_SOURCE_DIR}/external/NatNetLinux.git/include)

# create an executable
ADD_EXECUTABLE (optitrack_redis
	${CS225A_COMMON_SOURCE}
	${PROJECT_SOURCE_DIR}/src/optitrack/OptiTrackClient.cpp
	optitrack_main.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (optitrack_redis
	${CS225A_COMMON_LIBRARIES}
)
