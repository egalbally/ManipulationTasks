# glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

set (VISUALIZER_SOURCE
	uiforce/UIForceWidget.cpp
	visualizer_main.cpp)

add_executable(visualizer
	${CS225A_COMMON_SOURCE}
	${VISUALIZER_SOURCE})

target_link_libraries(visualizer
	${CS225A_COMMON_LIBRARIES}
	${GLFW_LIBRARY}
)
