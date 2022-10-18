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
  , m_velcroAspectRatio(0.085)
  , m_velcroARThreshold(0.03)
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
  //turn off for live debugging
  m_velcroAspectRatio = -1;
  m_velcroSize =-1;

  float dilation_size=1.0;
  m_morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );

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
  std::string req = "Incoming Request - Aspect Ratio: " + std::to_string(request->aspect_ratio) + " Size: " + std::to_string(request->size);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), req.c_str());
  m_velcroAspectRatio = request->aspect_ratio;
  m_velcroSize = request->size;

  geometry_msgs::msg::Pose velcroPos;
  processVelcro(velcroPos);
  response->centroid_pose = velcroPos;

}

void VelcroCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                                         const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA)
{
  m_colorImage = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
  m_depthImage = cv::Mat(cv_bridge::toCvCopy(depthImMsgA)->image);
  infoMsgA->header.frame_id;
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
  // geometry_msgs::msg::Pose velcroPos;
  // processVelcro(velcroPos); //live processing for debugging
}

void VelcroCentroid::processVelcro(geometry_msgs::msg::Pose &velcroPos)
{
  m_mask = 0;
  m_colorNames.createColorMask(m_colorImage, "black", m_mask);

  cv::Mat dilated, eroded;

  //erode first to delete smol noise outside obect of interest, dilate to restore object of interest to proper size
  erode(m_mask, eroded, m_morphology);
  dilate(eroded, dilated, m_morphology);

  //dilates to fill in noise inside object of interest and make it solid. erode to restore object to proper size
  dilate(dilated, dilated, m_morphology);
  erode(dilated, eroded, m_morphology);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(eroded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  for (size_t i =0; i < contours.size(); i++)
  {
    // draw Rotated Rect
    cv::RotatedRect rotRect = minAreaRect(contours[i]);

    //standardize the height and width regardless of orientation of strip or camera. height is the "longer portion of velcro"
    double height = std::max(rotRect.size.height,rotRect.size.width);
    double width = std::min(rotRect.size.height,rotRect.size.width);

    // if rotated rectangle aspect ratio is < desired aspect ratio, and the height is above service specified threshold
    if (m_velcroAspectRatio != -1 && m_velcroSize != -1)
    {
      // if aspect ratio is within threshold of what is expected.
      if ((width / height) < (m_velcroAspectRatio + m_velcroARThreshold/2) &&  (width / height) > (m_velcroAspectRatio - m_velcroARThreshold/2))
      {
        // if object meets min size requirement (close enough that it is not considered random noise)
        if(height > MIN_OBJECT_SIZE && width > MIN_OBJECT_SIZE/2 && height > (m_velcroSize-m_velcroSizeThreshold) && height < (m_velcroSize + m_velcroSizeThreshold))
        {   

          // calculate x,y coordinate of centroid
          cv::Moments moment = moments(contours[i]);
          cv::Point2f momentPt;
          if (moment.m00 != 0)
          { 

            momentPt = cv::Point2f(static_cast<float>(moment.m10 / moment.m00), static_cast<float>(moment.m01 / moment.m00));

            circle(m_colorImage, momentPt, 5, cv::Scalar(255, 255, 255), -1);
            std::string boxAR = "AR: " + std::to_string((width / height));
            std::string boxSize = "size: " + std::to_string(width) + " " + std::to_string(height) + " angle: " + std::to_string(rotRect.angle);
            putText(m_colorImage, boxAR, cv::Point2f(momentPt.x - 25, momentPt.y - 25),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            putText(m_colorImage, boxSize, cv::Point2f(momentPt.x - 25, momentPt.y - 10),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            //Depth image stuff
            std::string  depthPrint = "depth: " + std::to_string(m_depthImage.at<float>(momentPt)) + ", " + std::to_string(momentPt.x);
            putText(m_colorImage, depthPrint, cv::Point2f(momentPt.x - 25, momentPt.y + 20),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

            //set output for service call
            velcroPos.position.x = momentPt.x;
            velcroPos.position.y = momentPt.y;
            velcroPos.position.z = m_depthImage.at<float>(momentPt);
            tf2::Quaternion q;
            q.setRPY(rotRect.angle, 0, 0);
            velcroPos.orientation.x = q.x();
            velcroPos.orientation.y = q.y();
            velcroPos.orientation.z = q.z();
            velcroPos.orientation.w = q.w();
            std::string output = "Velcro found at " + std::to_string(velcroPos.position.x) + ", " + std::to_string(velcroPos.position.y) + ", " + std::to_string(velcroPos.position.z) + ", " + " angled: " + std::to_string(rotRect.angle) + "\n";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), output.c_str());
          }
        }
      }
    }
  }
  // cv::imshow("colornames", m_mask);
  // cv::imshow("dilate -> eroded", eroded);
  // cv::imshow("depth image", m_depthImage);
  // cv::waitKey(1);

  // cv::imshow("final result", m_colorImage);
  // cv::waitKey(1); //set to 1 for coninuous output, set to 0 for single frame forever

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
