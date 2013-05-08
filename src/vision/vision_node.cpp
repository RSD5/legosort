#include "ros/ros.h"
#include "vision_data.h"

void get_brick_callback(const ros::TimerEvent&)
{
  // Fetch bricks from the OpenCV Class.
  ROS_INFO("Fetching brick information.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle n;

  std::string all_bricks_publish_topic;
  n.param<std::string>("/Vision_NS/Vision_node/all_bricks_topic", all_bricks_publish_topic, "no_topic");

  double loop_interval;
  n.param<double>("/Vision_NS/Vision_node/loop_interval", loop_interval, 2.0);

  ros::Timer get_brick = n.createTimer(ros::Duration(loop_interval), get_brick_callback);

  ros::spin();

  return 0;
}