#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rc_node/car_input.h"
#include <sstream>


#if defined(OS_LINUX) || defined(OS_MACOSX)
#include <sys/ioctl.h>
#include <termios.h>
#elif defined(OS_WINDOWS)
#include <conio.h>
#endif

// Includes for HID protocol
#include "hid.hpp"
#include "stdint.h"

// HID buffers
uint8_t buf_in[64], buf_out[64];

#define KSTEERING_ANG_MAX 45.0   //Max steering dev. from center
#define KPOWER_MAX 1.0           //Max power value
#define KBITVAL_MAX_ABS 64       //Max potentiometer dev. from center

#define KSTEERING KBITVAL_MAX_ABS/KSTEERING_ANG_MAX
#define KPOWER KBITVAL_MAX_ABS/KPOWER_MAX

float clamp(float val, float min, float max);


// Callback for the car input from ROS
void inputCallback(const rc_node::car_input::ConstPtr& msg) {
  ROS_INFO("I heard: [%f, %f]", msg->steer_angle, msg->power);

  //Clamp the steering and throttle values
  float steer = clamp(msg->steer_angle, -KSTEERING_ANG_MAX, KSTEERING_ANG_MAX);
  float power = clamp(msg->power, -KPOWER_MAX, KPOWER_MAX);
  //first byte in the buffer is the value for the steering potentiometer
  buf_out[0] = (uint8_t) (steer*KSTEERING + KBITVAL_MAX_ABS);
  buf_out[1] = (uint8_t) (power*KPOWER + KBITVAL_MAX_ABS);
  rawhid_send(0, buf_out, 64, 100);  //Ignore errors for now... #TODO
  ROS_INFO("Sending out: [%d %d]", buf_out[0], buf_out[1]);
}

int main(int argc, char **argv) {
  // Setup the USB HID stuff (try the C example IDs first)
  int r = rawhid_open(1, 0x16C0, 0x0480, 0xFFAB, 0x0200);
  if (r <= 0) {
    //Next try the Arduino example IDs
    r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    if (r <= 0) {
      std::cout<<"No rc bridge device found\n";
      return -1;
    }
  }
  printf("Connected to rc bridge device!\n");

  // Initialize the ROS stuff and the node name
  ros::init(argc, argv, "rc_node");

  // Handle for all of the node interface stuff
  ros::NodeHandle n;

  // Subscribe to the topic that the steering angle and throttle are published on
  ros::Subscriber sub = n.subscribe("car_input", 10, inputCallback);

  ROS_INFO("Connected to RC Bridge device");

  // Just let the callbacks do their work
  ros::spin();

  return 0;
}

// Clamp a value between a min and max values
float clamp(float val, float min, float max) {
  if (val<min) {
    return min;
  } else if (val>max) {
    return max;
  } else {
    return val;
  }
}
