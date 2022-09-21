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
  colorNames.createColorMask(colorImage, "black", mask);

  cv::imshow("view", mask);
  cv::waitKey(10);

  cv::Mat dilated, eroded;
  float dilation_size=3.5;
  cv::Mat morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );
  dilate(mask, dilated, morphology);
  erode(dilated, eroded, morphology);

  cv::imshow("dilate -> eroded", eroded);
  cv::waitKey(10);

  cv::Mat imgContours;
  Canny(mask, imgContours, 30,200);

  cv::imshow("conts", imgContours);
  cv::waitKey(10);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(eroded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  cv::Mat res = cv::Mat::zeros (imgContours.size(), CV_8UC3);
  for (size_t i =0; i < contours.size(); i++)
  {
          // draw Rotated Rect
      cv::RotatedRect rotRect = minAreaRect(contours[i]);
      cv::Point2f vertices[4];
      rotRect.points(vertices);
      for (int i = 0; i < 4; i++)
          line(res, vertices[i], vertices[(i+1)%4], cv::Scalar(0,0,255), 2);

    //cv::Rect box = boundingRect(contours[i]);
    if ((rotRect.size.width/ rotRect.size.height) < 0.3 && rotRect.size.height > 300)
    {      
      //printf("Box[%ld] - %f, ", i, float(box.width)/ box.height);
      drawContours(res, contours, (int)i, cv::Scalar(0,255,0), 3, cv::LINE_8, hierarchy, 0);
      //printf("Contour %ld - %f/%f -> %f ---------", i, rotRect.size.width, rotRect.size.height , rotRect.size.width/rotRect.size.height);
      cv::Moments moment = moments(contours[i]);
      // calculate x,y coordinate of center
      cv::Point2f momentPt;
      if (moment.m00 != 0)
      {
        momentPt = cv::Point2f(static_cast<float>(moment.m10 / moment.m00), static_cast<float>(moment.m01 / moment.m00));
        circle(colorImage, momentPt, 5, cv::Scalar(255, 255, 255), -1);
        putText(colorImage, "  :D", cv::Point2f(momentPt.x - 25, momentPt.y - 25),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
      }

    }
  }
  //printf("\n");

  cv::imshow("res", res);
  cv::waitKey(10);

  cv::imshow("final", colorImage);
  cv::waitKey(10);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("velcro_centroid", options);

  //rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::AddTwoInts>("get_velcro_centroid", &add);

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("gripper/color/image_raw", 1, imageCallback);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  return 0;
}