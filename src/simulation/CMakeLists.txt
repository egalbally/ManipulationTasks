
add_executable(simulator
	${CS225A_COMMON_SOURCE}
	Simulator.cpp)

target_link_libraries(simulator
	${CS225A_COMMON_LIBRARIES})
