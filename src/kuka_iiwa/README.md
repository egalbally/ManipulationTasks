Kuka IIWA Redis Driver
======================

This driver was developped to work jointly with sai 2.0.
It works with redis. 
To use your own controller written in sai 2.0, you should use the same redis keys for the joint angles, joint velocities, and torques as the driver, as well as JSON formatting for the arrays.

usage
-----

1. Make sure that you are using the same redis keys in your controller as in the driver. By default, the keys you should read and write in the controller are :
	- "sai2::KUKA_IIWA::sensors::q"          Read the joint positions  
	- "sai2::KUKA_IIWA::sensors::dq"         Read the joint velocities
	- "sai2::KUKA_IIWA::sensors::torques"    Read the sensed torques (optionnal)
	- "sai2::KUKA_IIWA::actuators::fgc"      Write the commanded torques

2. Run the driver

		cd build
		./kuka_iiwa_redis_driver

3. Run the TorqueOverlay application on the robot

4. When the robot is floating (after it went back to home position), you can start your controller

5. When you are done, stop the driver (Ctrl+C) and your controller