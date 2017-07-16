// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.
// NOTE: this application only works in 2d.

#include <model/ModelInterface.h>
#include <simulation/SimulationInterface.h>
#include <graphics/GraphicsInterface.h>
#include <graphics/ChaiGraphics.h>
#include "redis/RedisClient.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "uiforce/UIForceWidget.h"
#include <iostream>
#include <string>
#include <cmath>

#define ENABLE_TRAJECTORIES

using namespace std;

static string world_file = "";
static string robot_file = "";
static string robot_name = "";
static const string CAMERA_NAME = "camera_fixed";

// redis keys: 
// NOTE: keys are formatted to be: REDIS_KEY_PREFIX::<robot-name>::<KEY>
static const string REDIS_KEY_PREFIX = "cs225a::robot::";
// - write:
static string JOINT_INTERACTION_TORQUES_COMMANDED_KEY = "::actuators::fgc_interact";
// - read:
static string JOINT_ANGLES_KEY        = "::sensors::q";
static string JOINT_VELOCITIES_KEY    = "::sensors::dq";

// function to parse command line arguments
static void parseCommandline(int argc, char** argv);

// callback to print glfw errors
static void glfwError(int error, const char* description);

// callback when a key is pressed
static void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
static void mouseClick(GLFWwindow* window, int button, int action, int mods);

// callback when user scrolls
static void mouseScroll(GLFWwindow* window, double xoffset, double yoffset);

// find the graphics object with the specified name inside the chai3d world.
static chai3d::cGenericObject *findObjectInWorld(chai3d::cWorld *world, const string &graphics_name);

// flags for scene camera movement
static bool fTransXp = false;
static bool fTransXn = false;
static bool fTransYp = false;
static bool fTransYn = false;
static bool fRotPanTilt = false;
static bool fZoom = false;
static double zoomSpeed = 0.0;
static bool fRobotLinkSelect = false;

static const HiredisServerInfo kRedisServerInfo = {
	"127.0.0.1",  // hostname
	6379,         // port
	{ 1, 500000 } // timeout = 1.5 seconds
};

#ifdef ENABLE_TRAJECTORIES
/********** Begin Custom Visualizer Code **********/

// Redis keys (read)
static string EE_POSITION_KEY         = "::tasks::ee_pos";
static string EE_POSITION_DESIRED_KEY = "::tasks::ee_pos_des";

// Chai3d graphics names
// - Created inside visualizer_main.cpp
static string EE_TRAJECTORY_CHAI_NAME         = EE_POSITION_KEY + "_traj";
static string EE_DESIRED_TRAJECTORY_CHAI_NAME = EE_POSITION_DESIRED_KEY + "_traj";
// - Created inside world.urdf
static string EE_POSITION_DESIRED_URDF_NAME   = EE_POSITION_DESIRED_KEY;

// Default number of points in trajectory buffer
static const int kLenTrajectory = 100;

// Minimum change between two points before updating trajectory
static const double kTrajectoryMinUpdateDistance = 0.005;

/**
 * Create a trajectory graphics object with len_trajectory points to be inserted
 * into chai3d::cWorld.
 */
static chai3d::cMultiSegment *createTrajectory(const string &graphics_name, const Eigen::Vector3d &starting_point) {
	// Create graphics object
	auto trajectory = new chai3d::cMultiSegment();
	trajectory->m_name = graphics_name;

	// Link together kLenTrajectory segments with vertices at starting_point
	trajectory->newVertex(starting_point(0), starting_point(1), starting_point(2));
	for (int i = 0; i < kLenTrajectory; i++) {
		trajectory->newVertex(starting_point(0), starting_point(1), starting_point(2));
		trajectory->newSegment(0, 0);
	}

	// Set trajectory color to white by default
	trajectory->setLineColor(chai3d::cColorf(1.0, 1.0, 1.0, 1.0));
	trajectory->setLineWidth(2.0);

	return trajectory;
}

/**
 * Add new point to trajectory and remove oldest point in trajectory buffer.
 */
static int updateTrajectoryPoint(chai3d::cMultiSegment *trajectory, int idx_traj, const Eigen::Vector3d &point) {
	int idx_traj_next = (idx_traj + 1) % kLenTrajectory;
	int idx_traj_next_2 = (idx_traj_next + 1) % kLenTrajectory;

	// Insert new point at next point in trajectory cycle
	trajectory->m_vertices->setLocalPos(idx_traj_next, point(0), point(1), point(2));
	trajectory->m_segments->setVertices(idx_traj, idx_traj, idx_traj_next);

	// Break the old connection to prevent a closed loop
	trajectory->m_segments->setVertices(idx_traj_next, idx_traj_next_2, idx_traj_next_2);
	return idx_traj_next;
}

/********** End Custom Visualizer Code **********/
#endif // ENABLE_TRAJECTORIES

int main(int argc, char** argv) {
	parseCommandline(argc, argv);
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = RedisClient();
	redis_client.serverIs(kRedisServerInfo);

	// load graphics scene
	auto graphics_int = new Graphics::GraphicsInterface(world_file, Graphics::chai, Graphics::urdf, true);
	Graphics::ChaiGraphics* graphics = dynamic_cast<Graphics::ChaiGraphics *>(graphics_int->_graphics_internal);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(CAMERA_NAME, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);

	// create a UI widget to apply mouse forces to the robot
	UIForceWidget force_widget(robot_name, robot.get(), graphics);
	force_widget.setEnable(false);

	/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS225a", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);
	glfwSetScrollCallback(window, mouseScroll);

	// cache and temp variables
	double last_cursorx, last_cursory;

	Eigen::VectorXd interaction_torques;

#ifdef ENABLE_TRAJECTORIES
	/********** Begin Custom Visualizer Code **********/

	// Set up trajectory tracking variables
	Eigen::Vector3d x, x_des;            // Current end effector pos
	Eigen::Vector3d x_prev, x_des_prev;  // Previous end effector pos
	int idx_traj = 0, idx_des_traj = 0;  // Current idx in trajectory buffer
	redis_client.getEigenMatrixDerivedString(EE_POSITION_KEY, x);
	redis_client.getEigenMatrixDerivedString(EE_POSITION_DESIRED_KEY, x_des);
	x_prev = x;
	x_des_prev = x_des;

	// Create trajectory graphics objects and insert them into the chai3d world
	auto x_traj = createTrajectory(EE_TRAJECTORY_CHAI_NAME, x);
	auto x_des_traj = createTrajectory(EE_DESIRED_TRAJECTORY_CHAI_NAME, x_des);
	x_des_traj->setLineColor(chai3d::cColorf(1.0, 0.0, 0.0, 1.0));  // Red for x_des
	graphics->_world->addChild(x_traj);
	graphics->_world->addChild(x_des_traj);

	// Retrieve cs225a::<robot_name>::tasks::ee_pos_des sphere marker from world.urdf
	auto x_des_marker = findObjectInWorld(graphics->_world, EE_POSITION_DESIRED_URDF_NAME);

	/********** End Custom Visualizer Code **********/
#endif // ENABLE_TRAJECTORIES

    // while window is open:
    while (!glfwWindowShouldClose(window))
	{
		// read from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

#ifdef ENABLE_TRAJECTORIES
		/********** Begin Custom Visualizer Code **********/

		redis_client.getEigenMatrixDerivedString(EE_POSITION_KEY, x);
		redis_client.getEigenMatrixDerivedString(EE_POSITION_DESIRED_KEY, x_des);

		// Update end effector position trajectory
		if ((x - x_prev).norm() > kTrajectoryMinUpdateDistance) {
			idx_traj = updateTrajectoryPoint(x_traj, idx_traj, x);
			x_prev = x;
		}

		// Update end effector desired position trajectory
		if ((x_des - x_des_prev).norm() > kTrajectoryMinUpdateDistance) {
			idx_des_traj = updateTrajectoryPoint(x_des_traj, idx_des_traj, x_des);
			x_des_prev = x_des;
		}

		// Update end effector desired position marker
		if (x_des_marker != nullptr) {
			x_des_marker->setLocalPos(chai3d::cVector3d(x_des));
		}

		/********** End Custom Visualizer Code **********/
#endif // ENABLE_TRAJECTORIES

		// update transformations
		robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot.get());
		graphics->render(CAMERA_NAME, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents();

	    // move scene camera as required
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			// camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
			// TODO: the above doesn't work as intended because Chai treats the lookat
			// vector as a direction vector in the local frame, rather than as a lookat point
			camera_pos = m_pan*(camera_pos);
			camera_lookat = m_pan*(camera_lookat);
			// TODO: the above fix is a HUGE hack. Think about improving this.
	    }
	    if (fZoom) {
			camera_pos = camera_pos + 0.04*camera_lookat*zoomSpeed;
			fZoom = false;
	    }
	    graphics->setCameraPose(CAMERA_NAME, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	    if (fRobotLinkSelect) {
			//activate widget
			force_widget.setEnable(true);
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);
			int viewx, viewy;
			viewx = floor(cursorx/wwidth_scr * wwidth_pix);
			viewy = floor(cursory/wheight_scr * wheight_pix);
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;
			if (cursorx > 0 && cursory > 0) {
				force_widget.setInteractionParams(CAMERA_NAME, viewx, wheight_pix-viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
	    } else {
			force_widget.setEnable(false);
	    }
		// get UI torques
		force_widget.getUIJointTorques(interaction_torques);
		//write to redis
		redis_client.setEigenMatrixDerivedString(JOINT_INTERACTION_TORQUES_COMMANDED_KEY, interaction_torques);
	}

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: visualizer <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	world_file = string(argv[1]);
	// argument 2: <path-to-robot.urdf>
	robot_file = string(argv[2]);
	// argument 3: <robot-name>
	robot_name = string(argv[3]);

	// Set up Redis keys
	JOINT_INTERACTION_TORQUES_COMMANDED_KEY = REDIS_KEY_PREFIX + robot_name + JOINT_INTERACTION_TORQUES_COMMANDED_KEY;
	JOINT_ANGLES_KEY        = REDIS_KEY_PREFIX + robot_name + JOINT_ANGLES_KEY;
	JOINT_VELOCITIES_KEY    = REDIS_KEY_PREFIX + robot_name + JOINT_VELOCITIES_KEY;

#ifdef ENABLE_TRAJECTORIES
	/********** Begin Custom Visualizer Code **********/

	EE_POSITION_KEY         = REDIS_KEY_PREFIX + robot_name + EE_POSITION_KEY;
	EE_POSITION_DESIRED_KEY = REDIS_KEY_PREFIX + robot_name + EE_POSITION_DESIRED_KEY;
	EE_TRAJECTORY_CHAI_NAME         = REDIS_KEY_PREFIX + robot_name + EE_TRAJECTORY_CHAI_NAME;
	EE_DESIRED_TRAJECTORY_CHAI_NAME = REDIS_KEY_PREFIX + robot_name + EE_DESIRED_TRAJECTORY_CHAI_NAME;
	EE_POSITION_DESIRED_URDF_NAME = REDIS_KEY_PREFIX + robot_name + EE_POSITION_DESIRED_URDF_NAME;

	/********** End Custom Visualizer Code **********/
#endif // ENABLE_TRAJECTORIES
}

//------------------------------------------------------------------------------

static void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

static void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

static void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			// TODO: move link select to shift + left click
			// TODO: set menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------
static void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
	fZoom = true;
	zoomSpeed = yoffset;
}

//------------------------------------------------------------------------------
static chai3d::cGenericObject *findObjectInWorld(chai3d::cWorld *world, const string &graphics_name) {
	for (unsigned int i = 0; i < world->getNumChildren(); ++i) {
		auto graphics_obj = world->getChild(i);
		if (graphics_obj->m_name == graphics_name) {
			return graphics_obj;
		}
	}
	return nullptr;
}
