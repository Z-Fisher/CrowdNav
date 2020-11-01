#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include <iostream>


void agent_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    ROS_INFO("I heard something");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gt_node");
    ros::NodeHandle n;
    ros::Subscriber agent_sub = n.subscribe("gazebo/model_states",
                                            1, 
                                            agent_callback);
    ros::spin();
    return 0;
}