Kuka IIWA Redis Driver
======================

This driver was developed to work jointly with sai 2.0.
It works with redis. 
To use your own controller written in sai 2.0, you should use the same redis keys for the joint angles, joint velocities, and torques as the driver.

Installation
------------

1. Obtain FRI-Client-SDK and KukaLBRDynamics and put them in the src/kuka_iiwa directory.

2. Build the FRI library.

   ```cd cs225a.git/src/kuka_iiwa/FRI-Client-SDK/build/GNUMake```
   ```make```

3. Uncomment the following line cs225a.git/CMakeLists.txt file to build the driver.

   ```add_subdirectory(src/kuka_iiwa)```

4. Make

   ```./make.sh```

Usage
-----

1. Make sure that you are using the same redis keys in your controller as in the driver. The keys can be imported with:

   ```#include "kuka_iiwa/RedisClient.h"```

   By default, the keys you should read and write in the controller are:
	- "cs225a::kuka_iiwa::sensors::q"          Read the joint positions
	- "cs225a::kuka_iiwa::sensors::dq"         Read the joint velocities
	- "cs225a::kuka_iiwa::sensors::torques"    Read the sensed torques (optional)
	- "cs225a::kuka_iiwa::actuators::fgc"      Write the commanded torques

2. Make sure your tool.xml file specifies the correct weight of your end-effector.

3. Run the driver

   ```
   cd bin
   ./make.sh
   ./kuka_iiwa_driver
   ```

4. Run the TorqueOverlay application on the robot WHILE KEEPING YOUR HAND ON THE E-STOP.

5. When the robot is floating (after it went back to home position), you can start your controller

6. When you are done, stop the driver (Ctrl+C) and your controller
