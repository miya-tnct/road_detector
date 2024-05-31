#include "road_detector/road_detector_node.h"

#include "cv_bridge/cv_bridge.h"
#include "ros/this_node.h"
#include "std_msgs/Float64.h"

namespace road_detector
{

RoadDetectorNode::RoadDetectorNode()
: ros::NodeHandle()
, threshold_(30000)
, road_error_publisher_(this->advertise<std_msgs::Float64>("road_error", 1, false))
, camera_image_subscriber_(this->subscribe("camera/color/image_raw", 1, &RoadDetectorNode::updateRoad, this))
{
}

void RoadDetectorNode::updateRoad(sensor_msgs::Image::ConstPtr camera_image_msg)
try
{
  const auto cv_ptr = cv_bridge::toCvCopy(camera_image_msg, sensor_msgs::image_encodings::BGR8);
  const auto & img = cv_ptr->image;
  const auto rows = img.rows;
  const auto cols = img.cols;
  long bright_x_total = 0;
  int bright_count = 0;
  for (int y = 0; y < cols; ++y)
  {
    const auto row_ptr = img.ptr<cv::Vec3b>(y);
    for (int x = 0; x < rows; ++ x)
    {
      const auto & picel = row_ptr[x];
      const auto brightness
        = static_cast<int>(picel[0]) * picel[0]
        + static_cast<int>(picel[1]) * picel[1]
        + static_cast<int>(picel[2]) * picel[2];
      if (brightness > threshold_)
      {
        bright_x_total += x;
        ++bright_count;
      }
    }
  }

  const auto bright_x_avg = bright_x_total / bright_count;
  const auto row_centor = rows / 2;

  std_msgs::Float64 road_error_msg;
  road_error_msg.data = bright_x_avg - row_centor;

  road_error_publisher_.publish(road_error_msg);
}
catch(std::exception e)
{
  ROS_WARN("%s: %s", ros::this_node::getName().c_str(), e.what());
}

}