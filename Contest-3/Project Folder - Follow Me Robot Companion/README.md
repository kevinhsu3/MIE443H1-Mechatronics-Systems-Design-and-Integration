# mie443_contest3

#For sim:
#roslaunch turtlebot_gazebo turtlebot_world.launch
#rosrun sound_play soundplay_node.py

#For robot
roslaunch turtlebot_bringup minimal.launch
rosrun sound_play soundplay_node.py
roslaunch turtlebot_follower follower.launch

# mie443_contest3 code outline

Tuesday group 3

This package contains a code to run a "follow-me companion" turtlebot which displays emotions when reacting to specific stimuli. The code is implemented with a C++ ROS script and utilizes all the basic ROS libraries included in the package.

The robot is controlled by a state machine and has a default state to follow the user. The "follower.cpp" must be included when running the code to properly run the follower state.

The turtlebot has 4 reaction states:
  Showing an image of a can of tuna leads the turtlebot to show "positively excited"
  Kicking the left or right bumper leads to "surprise"
  Losing track of the user leads to "fear"
  Being blocked from the user by an obsacle leads to "rage"

To implement the code the computer must be connected to the turtlebot and the following commands run in the terminator in their individual windows:
  catkin_make
  roslaunch turtlebot_bringup minimal.launch
  rosrun sound_play soundplay_node.py
  roslaunch turtlebot_follower follower.launch
  rosrun mie443_contest3 contest3

*BEFORE EXECUTING THE CODE ENSURE THAT THE PERSON YOU INTEND THE ROBOT TO BE FOLLOWING IS STANDING IN FRONT OF THE ROBOT AT THE REQUIRED FOLLOWING DISTANCE OF 1 METER*
