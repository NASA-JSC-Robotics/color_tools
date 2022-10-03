#include "velcro-centroid/VelcroCentroid.h"
#define MIN_OBJECT_SIZE 25

using std::placeholders::_1;
using std::placeholders::_2;


// save the image to a member and then process the image with what comes in the service call.

//for future streaming purposes, you would need to set up a multithreaded executor, put a wait in service, and then separate both subscribers into unique callback groups

VelcroCentroid::VelcroCentroid()
  : rclcpp::Node("velcro_centroid")
  , m_velcroSize(100)
  , m_velcroSizeThreshold(200)
  , m_velcroAspectRatio(0.3)
  , m_velcroARThreshold(0.55)
  , m_imageQos(1)
{
  initialize();
}

VelcroCentroid::~VelcroCentroid(){}

void VelcroCentroid::initialize()
{
  m_imageQos.keep_last(10);
  m_imageQos.reliable();
  m_imageQos.durability_volatile();
  m_velcroAspectRatio = -1;
  m_velcroSize =-1;

  service_ = this->create_service<perception_msgs::srv::VelcroDimensions>("set_velcro_dimensions", std::bind(&VelcroCentroid::set_velcro_dimensions, this, _1, _2));
  m_imageSub = image_transport::create_subscription(this, "gripper/color/image_raw", std::bind(&VelcroCentroid::imageCallback, this, _1),"raw",m_imageQos.get_rmw_qos_profile());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to process images on service request ");
}

void VelcroCentroid::set_velcro_dimensions(const std::shared_ptr<perception_msgs::srv::VelcroDimensions::Request> request,
          std::shared_ptr<perception_msgs::srv::VelcroDimensions::Response>      response)
{
  response->centroid_pose.position.x = request->aspect_ratio + request->size;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nasp ratio: %f" " size: %f",
                request->aspect_ratio, request->size);
  m_velcroAspectRatio = request->aspect_ratio;
  m_velcroSize = request->size;
  //processVelcro();
}

void VelcroCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & colorImMsgA)
{
  m_colorImage = cv::Mat(cv_bridge::toCvShare(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
  processVelcro(); //tennis ball demo
}

void VelcroCentroid::processVelcro()
{
  static cv::Mat mask;

  ColorNames colorNames;
  colorNames.createColorMask(m_colorImage, "black", mask);

  cv::imshow("view", mask);

  cv::Mat dilated, eroded;
  float dilation_size=5.5;
  cv::Mat morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );
  dilate(mask, dilated, morphology);
  erode(dilated, eroded, morphology);

  cv::imshow("dilate -> eroded", eroded);

  cv::Mat imgContours;
  Canny(mask, imgContours, 30,200);

  cv::imshow("conts", imgContours);

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
    // if rotated rectangle aspect ratio is < desired aspect ratio, and the height is above service specified threshold
    if (m_velcroAspectRatio != -1 && m_velcroSize != -1)
    {
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "min AR: %f" " max AR: %f",
      //   (m_velcroAspectRatio - m_velcroARThreshold/2) , (m_velcroAspectRatio + m_velcroARThreshold/2));
      //if ((rotRect.size.width/ rotRect.size.height) < (m_velcroAspectRatio + m_velcroARThreshold/2) &&  (rotRect.size.width/ rotRect.size.height) > (m_velcroAspectRatio - m_velcroARThreshold/2))
      if(true)
      {
        //TODO: add in min max  boundaries
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "min size: %f" " max size: %f",
        //         m_velcroSize - m_velcroSizeThreshold, m_velcroSize + m_velcroSizeThreshold);
        //if(rotRect.size.height > (m_velcroSize - m_velcroSizeThreshold) && rotRect.size.height < (m_velcroSize + m_velcroSizeThreshold) )
        if(true)
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
            circle(m_colorImage, momentPt, 5, cv::Scalar(255, 255, 255), -1);
            std::string boxAR = "AR: " + std::to_string((rotRect.size.width/ rotRect.size.height));
            std::string boxSize = "size: " + std::to_string(rotRect.size.height );
            putText(m_colorImage, boxAR, cv::Point2f(momentPt.x - 25, momentPt.y - 25),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            putText(m_colorImage, boxSize, cv::Point2f(momentPt.x - 25, momentPt.y - 10),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
          }
        }
      }
    }
  }
  //printf("\n");

  cv::imshow("res", res);

  cv::imshow("final", m_colorImage);
  cv::waitKey(1); //set to 1 for coninuous output, set to 0 for single frme forever

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VelcroCentroid>());
  rclcpp::shutdown();
  cv::destroyAllWindows();

  return 0;
}
