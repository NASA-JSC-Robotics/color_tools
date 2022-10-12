#include "velcro-centroid/VelcroCentroid.h"
#define MIN_OBJECT_SIZE 25

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


// save the image to a member and then process the image with what comes in the service call.

//for future streaming purposes, you would need to set up a multithreaded executor, put a wait in service, and then separate both subscribers into unique callback groups

VelcroCentroid::VelcroCentroid()
  : rclcpp::Node("velcro_centroid")
  , m_velcroSize(372)
  , m_velcroSizeThreshold(50)
  , m_velcroAspectRatio(0.06)
  , m_velcroARThreshold(0.1)
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
  //m_velcroAspectRatio = -1;
  //m_velcroSize =-1;

  service_ = this->create_service<perception_msgs::srv::VelcroDimensions>("set_velcro_dimensions", std::bind(&VelcroCentroid::set_velcro_dimensions, this, _1, _2));

  m_depthImageSub.subscribe(this, "gripper/aligned_depth_to_color/image_raw", m_imageQos.get_rmw_qos_profile());
  m_colorImageSub.subscribe(this, "gripper/color/image_raw", m_imageQos.get_rmw_qos_profile());
  m_colorInfoSub.subscribe(this, "gripper/color/camera_info", m_imageQos.get_rmw_qos_profile());

  m_timeSyncPtr = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                        sensor_msgs::msg::CameraInfo>>(m_colorImageSub, m_depthImageSub, m_colorInfoSub, 10);
  m_timeSyncPtr->registerCallback(std::bind(&VelcroCentroid::imageCallback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));
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

void VelcroCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                                         const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA)
{
  m_colorImage = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
  m_depthImage = cv::Mat(cv_bridge::toCvCopy(depthImMsgA)->image);    
  if(m_depthImage.type() != CV_32FC1)
  {
      if(m_depthImage.type() == CV_16UC1)
      {
          cv::Mat(m_depthImage).convertTo(m_depthImage, CV_32FC1, 0.001);
      }
      else
      {
          printf("%s depth image type must be CV_32FC1 or CV_16UC1\n", __FUNCTION__);
      }
  }
  processVelcro(); //tennis ball demo
}

void VelcroCentroid::processVelcro()
{
  //cv::imshow("pre-process", m_colorImage);

  m_mask = 0;
  m_colorNames.createColorMask(m_colorImage, "black", m_mask);


  cv::imshow("colornames", m_mask);

  cv::Mat dilated, eroded;
  float dilation_size=1.0;
  cv::Mat morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );
  //erodes first to delete smol blips
  erode(m_mask, eroded, morphology);
  dilate(eroded, dilated, morphology);

  dilation_size = 1.5;
  morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );

  //dilates and then erodes to patch holes in in blobs
  dilate(dilated, dilated, morphology);
  erode(dilated, eroded, morphology);

  cv::imshow("dilate -> eroded", eroded);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::Mat res = eroded;
  findContours(eroded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);


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
      if ((rotRect.size.width/ rotRect.size.height) < (m_velcroAspectRatio + m_velcroARThreshold/2) &&  (rotRect.size.width/ rotRect.size.height) > (m_velcroAspectRatio - m_velcroARThreshold/2))
      {
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
            //Depth image stuff
            //cv::Point depth= cv::Point(int(floor(momentPt.x)),int(floor(momentPt.x)));
            std::string  depthPrint = "depth: " + std::to_string(m_depthImage.at<float>(momentPt));
            putText(m_colorImage, depthPrint, cv::Point2f(momentPt.x - 25, momentPt.y + 20),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
          }
        }
      }
    }
  }
  //printf("\n");

  cv::imshow("res", res);
  cv::waitKey(1);

  cv::imshow("depth", m_depthImage);
  cv::waitKey(1);

  cv::imshow("final", m_colorImage);
  cv::waitKey(1); //set to 1 for coninuous output, set to 0 for single frme forever

}


int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "opencv : %s\n", CV_VERSION);
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VelcroCentroid>());
  rclcpp::shutdown();
  cv::destroyAllWindows();

  return 0;
}
