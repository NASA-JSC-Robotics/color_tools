#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <color_names/ColorNames.h>

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & colorImMsgA)
{
  static cv::Mat mask;
  cv::Mat colorImage = cv::Mat(cv_bridge::toCvShare(colorImMsgA, "bgr8")->image);    // this is the opencv encoding

  ColorNames colorNames;
  colorNames.createColorMask(colorImage, "brown", mask);

  cv::imshow("view", mask);
  cv::waitKey(10);


  cv::Mat imgContours;
  Canny(mask, imgContours, 30,200);

  cv::imshow("conts", imgContours);
  cv::waitKey(10);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(imgContours, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  cv::Mat res = cv::Mat::zeros (imgContours.size(), CV_8UC3);
  for (size_t i =0; i < contours.size(); i++)
  {
    cv::Rect box = boundingRect(contours[i]);
    if (float(box.width)/ box.height < 0.4)
    {
      printf("Box[%ld] - %f, ", i, float(box.width)/ box.height);
      drawContours(res, contours, (int)i, cv::Scalar(0,255,0), 3, cv::LINE_8, hierarchy, 0);

    }
      
  }


  cv::imshow("res", res);
  cv::waitKey(10);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("gripper/color/image_raw", 1, imageCallback);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  return 0;
}