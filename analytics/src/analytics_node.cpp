#include "ros/ros.h"	
#include "gazebo_msgs/ModelStates.h"	
#include "geometry_msgs/Pose.h"	
#include "geometry_msgs/Twist.h"	
#include <iostream>	
#include <string> 	
#include <nav_msgs/Odometry.h>
#include <chrono>
#include "visualization_msgs/MarkerArray.h"
#include "array"


bool first_time = true;
bool first_goal = true;
auto start = std::chrono::high_resolution_clock::now();
float dist_to_goal = 1000.0;
float prev_x = 1000.0;
float prev_y = 1000.0;
float total_dist = 0.0;
float goal_x = 0.0;
float goal_y = 0.0;


// Calculates distance traveled by robot
// Calculates time spent stopped
void pose_cb(const nav_msgs::Odometry& msg) {	

    float x = msg.pose.pose.position.x;
    float y = msg.pose.pose.position.y;
    float dist_x = 0.0;
    float dist_y = 0.0;

    if (prev_x == 1000.0) {
        prev_x = x;
    } else {
        dist_x = x - prev_x;
        prev_x = x;
    }
    if (prev_y == 1000.0) {
        prev_y = y;
    } else {
        dist_y = y - prev_y;
        prev_y = y;
    }
    if (pow(dist_x, 2) > 0.0 || pow(dist_y, 2) > 0.0) {
        total_dist += sqrt(pow(dist_x,2) + pow(dist_y,2));
    }
}	

// Gets remaining distance to goal
void goal_cb(const visualization_msgs::MarkerArray& msg) {	

    // initial dist_to_goal is incorrect but as long as 
    // sim runs for a couple iterations, will be OK
    if (first_goal) {
        goal_x = msg.markers[0].pose.position.x;
        goal_y = msg.markers[0].pose.position.y;
        first_goal = false;
    }

    float dist_x = goal_x - prev_x;
    float dist_y = goal_y - prev_y;
    dist_to_goal = sqrt(pow(dist_x,2) + pow(dist_y,2));
}	


//TODO ped callback func to monitor proximity / collisions


int main(int argc, char **argv) {	

    if (first_time) {
        auto start = std::chrono::high_resolution_clock::now();
        first_time = false;
    }

    ros::init(argc, argv, "analytics_node");	
    ros::NodeHandle n;	

    ros::Subscriber pose_sub = n.subscribe("/pedsim_simulator/robot_position",	
                                            1, 	
                                            pose_cb);	

    ros::Subscriber goal_sub = n.subscribe("/robot/goal",	
                                            1, 	
                                            goal_cb);

    // TODO: ped sub



    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();

        if (dist_to_goal <= 15) {
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            ROS_INFO("DONE - Sufficiently close to goal");
            ROS_INFO("Time taken: %f", elapsed.count());
            ROS_INFO("Total distance of path traveled: %f", total_dist);
            ros::shutdown();
        }
        //rate.Sleep();
    }
    //ros::spin();	
    return 0;	
} 