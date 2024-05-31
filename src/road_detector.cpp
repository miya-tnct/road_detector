#include "ros/init.h"
#include "ros/spinner.h"

#include "road_detector/road_detector_node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "road_detector");
  road_detector::RoadDetectorNode node;
  ros::spin();
  return 0;
}