#include <ros/ros.h>
#include "cs/main/debug_pub_wrapper.h"
#include "cs/main/state_machine.h"
#include "config_reader/config_reader.h"
#include <string>
#include <control_stack/Obstacles.h>

namespace params {
CONFIG_STRING(map, "pf.map");
}


void obstacle_callback(const control_stack::Obstacles::ConstPtr& msg) {
  //ROS_INFO("num peds: [%f]", msg->circles[0]);
  ROS_INFO("x_pos: [%f]", msg->circles[0].center.x);
  ROS_INFO("y_pos: [%f]", msg->circles[0].center.y);
  ROS_INFO("x_vel: [%f]", msg->circles[0].velocity.x);
  ROS_INFO("y_vel: [%f]", msg->circles[0].velocity.y);
  ROS_INFO("ped_radius: [%f]", msg->circles[0].radius);
}

// TODO struct of peds with info (circle info only)


int main(int argc, char** argv) {

  // initialize ros and node
  const std::string pub_sub_prefix = "/robot";
  ros::init(argc, argv, "nav_node");
  ros::NodeHandle n;

  // config file
  std::string config_file = "src/CrowdNav/control_stack/config/nav_config.lua";
  n.getParam("/nav_node/config_file_path", config_file);      
  config_reader::ConfigReader reader({config_file});

  // setup state machine
  cs::main::DebugPubWrapper dpw(&n, pub_sub_prefix);
  cs::main::StateMachine state_machine(&dpw, &n, pub_sub_prefix);
  
  // setup laser sub subscriber
  ros::Subscriber laser_sub =
      n.subscribe("/scan",
                  1,
                  &cs::main::StateMachine::UpdateLaser,
                  &state_machine);
                  
  ros::Subscriber obstacle_sub =
    n.subscribe("/obstacles",
                1,
                &cs::main::StateMachine::UpdatePeds,
                &state_machine);


  // setup command publisher
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>(
      constants::kCommandVelocityTopic, 1);

  RateLoop rate(40.0);
  while (ros::ok()) {
    ros::spinOnce();
    command_pub.publish(state_machine.ExecuteController().ToTwist());
    rate.Sleep();
  }
  return 0;
}
