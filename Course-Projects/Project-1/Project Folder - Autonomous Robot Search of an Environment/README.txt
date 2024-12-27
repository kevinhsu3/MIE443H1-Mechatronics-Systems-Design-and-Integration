MIE443 Contest 1 Code Outline


Tuesday Group 3


This package contains a code to run a turtlebot autonomously through an unknown environment and map the given region, including all obstacles and walls. The code is implemented with a C++ ROS script and utilizes all the basic ROS libraries included in the package. Any additional libraries included are only C++ standard libraries, ie. <stdlib.h>. 


The robot is controlled by a state machine and has a default navigation logic to switch between these different states which are defined by various movement functions. The switch between states is triggered based on sensory data (from bumpers, odometry, and a laser scan) when the robot gets close to or hits objects. The main functions and forms of navigation include: spinning around and orienting towards space, turning to avoid obstacles which are sensed, turning to get unstuck if an object is hit, and moving forward in a straight line.


The main function only includes a switch statement for states. State switches and navigational logic are both done inside state functions. The code was designed around everything being non-blocking, so the only logic performed in states are state updates and state switches. As such, there is heavy use of global variables to remember information and state variables between ROS loops. The most important of these are the variables state, stepNo, and dynVar, which keep track of the state, state progress, and any dynamically remembered information. These variables are cleared whenever the robot changes states. Currently dynVar has a max index of 9 and is statically assigned. 


The most used function in the code is turn(float turnAmt, int ranIdx). This function is pivotal to the operation of the robot, it functions as a non-blocking sub-state to accurately turn the robot a fixed angle. It does this by assigning the starting yaw to dynVar to remember between ROS loops, then turning until it reaches the desired yaw. If multiple turns are called in succession, the index of dynVar is subsequently increased so memory from previous turns does not interfere.