#include "ros/ros.h"
#include <string>
#include "vision_data.h"
#include "legosort/RawBrick.h"
#include <opencv2/opencv.hpp>
#include "brick_finding_class.cpp"


BrickDetect Detector;
ros::Publisher raw_bricks_pub;

void get_brick_callback(const ros::TimerEvent&)
{
  // Fetch bricks from the OpenCV Class.
  // ROS_INFO("Fetching brick information.");
  vector<Brick> bricks;
  bricks = Detector.get_bricks();
  legosort::RawBrick msg;
  for ( unsigned int i = 0; i < bricks.size(); i++) {
    // if (bricks[i].color == YELLOW) {
    //   std::cout << "Yellow brick at ";
    // }
    // if (bricks[i].color == BLUE) {
    //   std::cout << "BLUE brick at ";
    // }
    // if (bricks[i].color == RED) {
    //   std::cout << "RED brick at ";
    // }

    // std::cout << bricks[i].x_coord << " x " << bricks[i].y_coord << ". Angle: " << bricks[i].angle << std::endl;
    msg.timestamp = bricks[i].timestamp;
    msg.color = bricks[i].color;
    msg.x_coord = bricks[i].x_coord;
    msg.y_coord = bricks[i].y_coord;
    msg.angle = bricks[i].angle;
    msg.width = bricks[i].width;
    msg.height = bricks[i].height;

    raw_bricks_pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle n;

  // Parameters
  std::string all_bricks_publish_topic;
  n.param<std::string>("/Vision_NS/Vision_node/all_bricks_topic", all_bricks_publish_topic, "no_topic");

  double loop_interval;
  n.param<double>("/Vision_NS/Vision_node/loop_interval", loop_interval, 2.0);

  // Setup the detector
  Detector.set_cam(0);
  Detector.start_cam();

  // Publisher
  raw_bricks_pub = n.advertise<legosort::RawBrick>(all_bricks_publish_topic, 10);

  ros::Timer get_brick = n.createTimer(ros::Duration(loop_interval), get_brick_callback);

  ros::spin();

  return 0;
}