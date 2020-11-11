#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string> 


void modelstate_callback(const gazebo_msgs::ModelStates& msg) {
    for (int i = 0; i < msg.name.size(); i++) {
        std::string agent_name = msg.name[i];
        geometry_msgs::Twist agent_twist = msg.twist[i];
        geometry_msgs::Pose agent_pose = msg.pose[i];
        try {
            int agent_num = std::stoi(agent_name);
            // do some other stuff here
        } catch (const std::invalid_argument& ia) {
            continue;
        }
        ROS_INFO("Agent name: %s", agent_name.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gt_node");
    ros::NodeHandle n;
    ros::Subscriber agent_sub = n.subscribe("gazebo/model_states",
                                            1, 
                                            modelstate_callback);
    ros::spin();
    return 0;
}