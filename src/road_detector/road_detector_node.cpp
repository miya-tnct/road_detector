#include "road_detector/road_detector_node.h"

#include "cv_bridge/cv_bridge.h"
#include "ros/this_node.h"
#include "std_msgs/Float64.h"
#include "opencv4/opencv2/highgui.hpp"

namespace road_detector
{

RoadDetectorNode::RoadDetectorNode()
: ros::NodeHandle()
, threshold_(60000)
, deviation_publisher_(this->advertise<std_msgs::Float64>("deviation", 1, false))
, camera_image_subscriber_(this->subscribe("usb_cam/image_raw", 1, &RoadDetectorNode::updateRoad, this))
{
}

void RoadDetectorNode::updateRoad(sensor_msgs::Image::ConstPtr camera_image_msg)
try
{
  const auto cv_ptr = cv_bridge::toCvCopy(camera_image_msg, sensor_msgs::image_encodings::BGR8);
  const auto & img = cv_ptr->image;
  auto img_cpy = img.clone();
  const auto rows = img.rows;
  const auto cols = img.cols;
  const auto x_centor = cols / 2;
  long bright_x_total = 0;
  int bright_count = 0;
  for (int y = 0; y < rows; ++y)
  {
    const auto row_ptr = img.ptr<cv::Vec3b>(y);
    auto cpy_row_ptr = img_cpy.ptr<cv::Vec3b>(y);
    for (int x = 0; x < cols; ++ x)
    {
      const auto & picel = row_ptr[x];
      auto & cpy_pic = cpy_row_ptr[x];
      const auto brightness
        = static_cast<int>(picel[0]) * picel[0]
        + static_cast<int>(picel[1]) * picel[1]
        + static_cast<int>(picel[2]) * picel[2];
      if (brightness >= threshold_)
      {
        cpy_pic[0] = 255;
        cpy_pic[1] = 255;
        cpy_pic[2] = 255;
        bright_x_total += x;
        ++bright_count;
      }
      else {
        cpy_pic[0] = 0;
        cpy_pic[1] = 0;
        cpy_pic[2] = 0;
      }
    }
  }


  cv::imshow("aaa", img_cpy);
 cv::waitKey(1);
  if (!bright_count) {
    return;
  }
  const auto bright_x_avg = bright_x_total / bright_count;

  std_msgs::Float64 deviation_msg;
  deviation_msg.data = bright_x_avg - x_centor;

  deviation_publisher_.publish(deviation_msg);
}
catch(std::exception e)
{
  ROS_WARN("%s: %s", ros::this_node::getName().c_str(), e.what());
}

}