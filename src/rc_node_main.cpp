#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rc_node/car_input.h"
#include <sstream>

void inputCallback(const rc_node::car_input::ConstPtr& msg) {
  ROS_INFO("I heard: [%f, %f]", msg->steer_angle, msg->power);
}

int main(int argc, char **argv) {
  // Initialize the ROS stuff and the node name
  ros::init(argc, argv, "rc_node");

  // Handle for all of the node interface stuff
  ros::NodeHandle n;

  // Subscribe to the topic that the steering angle and throttle are published on
  ros::Subscriber sub = n.subscribe("car_input", 10, inputCallback);

  // Just let the callbacks do their work
  ros::spin();

  return 0;
}
