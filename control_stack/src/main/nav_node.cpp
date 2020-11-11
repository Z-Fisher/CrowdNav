#include <ros/ros.h>
#include "cs/main/debug_pub_wrapper.h"
#include "cs/main/state_machine.h"
#include "config_reader/config_reader.h"
#include <string>


namespace params {
CONFIG_STRING(map, "pf.map");
}

int main(int argc, char** argv) {
  // config file
  std::string config_file = "src/CrowdNav/control_stack/config/nav_config.lua";
  config_reader::ConfigReader reader({config_file});

  // initialize ros and node
  const std::string pub_sub_prefix = "/robot";
  ros::init(argc, argv, "nav_node");
  ros::NodeHandle n;
  
  // setup state machine
  cs::main::DebugPubWrapper dpw(&n, pub_sub_prefix);
  cs::main::StateMachine state_machine(&dpw, &n, pub_sub_prefix);
  
  // setup ground truth subscriber as alternate to perceptual data
  ros::Subscriber agent_sub = n.subscribe("gazebo/model_states",
                                          1, 
                                          &cs::main::StateMachine::UpdateGT,
                                          &state_machine);
  
  /*
  // setup odometry subscriber
  ros::Subscriber odom_sub = n.subscribe(pub_sub_prefix + constants::kOdomTopic,
                                         1,
                                         &cs::main::StateMachine::UpdateOdom,
                                         &state_machine);
  
  // setup laser sub subscriber
  ros::Subscriber laser_sub =
      n.subscribe(pub_sub_prefix + constants::kLaserTopic,
                  1,
                  &cs::main::StateMachine::UpdateLaser,
                  &state_machine);
  */

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
