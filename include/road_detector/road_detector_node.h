#ifndef ROAD_DETECTOR_ROAD_DETECTOR_NODE_H
#define ROAD_DETECTOR_ROAD_DETECTOR_NODE_H

#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"

namespace road_detector
{

class RoadDetectorNode : public ros::NodeHandle
{
public:
  RoadDetectorNode();

private:
  void updateRoad(sensor_msgs::Image::ConstPtr camera_image_msg);

  int threshold_;

  ros::Publisher road_error_publisher_;
  ros::Subscriber camera_image_subscriber_;
};

}

#endif