// UiForceWidget.h

#ifndef UIFORCE_WIDGET_H
#define UIFORCE_WIDGET_H

#include <model/ModelInterface.h>
#include <graphics/ChaiGraphics.h>
#include <chai3d.h>
#include <string>
#include <Eigen/Core>

class UIForceWidget {
public:
	enum UIForceWidgetState {
		Disabled = 0,
		Inactive,
		Active
	};

public:
	//ctor
	UIForceWidget(const std::string& robot_name, Model::ModelInterface* robot, Graphics::ChaiGraphics* graphics)
	: _robot_name(robot_name), _robot(robot), _graphics(graphics) {
		_display_line = new chai3d::cShapeLine();
		_display_line->setShowEnabled(false);
		_graphics->_world->addChild(_display_line);
		//TODO: set default line display properties

		// set state to inactive initially
		_state = Inactive;

		//TODO: update defaults parameters below
		_max_force = 20; // N
		_spring_k = 20; // N/m
	}

	// set state
	void setEnable(bool enable);

	// get state
	UIForceWidgetState getState() const {return _state;}

	// set current window and cursor properties
	// this updates the internal parameters for calculating the ui interaction force
	void setInteractionParams(const std::string& camera_name,
								int viewx,
								int viewy,
								int window_width,
								int window_height);

	// get interaction force
	void getUIForce(Eigen::Vector3d& ret_force) const;

	// get interaction joint torques
	void getUIJointTorques(Eigen::VectorXd& ret_torques) const;

	//dtor: 
	~UIForceWidget() {
		delete _display_line;
	}

public:
	// a line to be displayed when an interaction force is applied
	chai3d::cShapeLine* _display_line;

	// name of robot
	std::string _robot_name;

	// robot model this UIForceWidget is associated with
	Model::ModelInterface* _robot;

	// handle to graphics interface to query interaction state change
	Graphics::ChaiGraphics* _graphics;

	// current state of the widget
	UIForceWidgetState _state;

	// spring constant to use to calculate force
	double _spring_k;

	// maximum allowable force
	double _max_force;

	// link on which force is being applied
	std::string _link_name;

	// local position at which force is being applied
	Eigen::Vector3d _link_local_pos;
};

#endif //UIFORCE_WIDGET_H