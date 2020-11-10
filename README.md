
Clone files to your ~/ws/src

$ gedit ~/.bashrc

$ export TURTLEBOT3_MODEL=waffle_pi

Save and close ~/.bashrc

$ source ~/.bashrc

$ cd ~/ws

$ catkin_make

$ source devel/setup.bash

Ensure that you are in the ws level of your directory

$ roslaunch pedsim_simulator combo.launch 

$ rosrun control_stack nav_node
