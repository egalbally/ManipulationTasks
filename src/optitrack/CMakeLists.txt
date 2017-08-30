# create an executable
ADD_EXECUTABLE (optitrack_redis
	${CS225A_COMMON_SOURCE}
	optitrack_main.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (optitrack_redis
	${CS225A_COMMON_LIBRARIES}
)
