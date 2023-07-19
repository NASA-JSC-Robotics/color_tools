#include "color_blob_centroid/ColorBlobCentroid.h"
#define MIN_OBJECT_SIZE 18

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


// save the image to a member and then process the image with what comes in the service call.

//for future streaming purposes, you would need to set up a multithreaded executor, put a wait in service, and then separate both subscribers into unique callback groups

ColorBlobCentroid::ColorBlobCentroid()
  : rclcpp::Node("color_blob_centroid")
  , m_blobSize(-1)
  , m_blobSizeThreshold(50)
  , m_blobAspectRatio(-1)
  , m_blobARThreshold(0.03)
  , m_imageQos(1)
  , m_color("red")
  , m_continuousColor(false)
{
  initialize();
}

ColorBlobCentroid::~ColorBlobCentroid(){}

void ColorBlobCentroid::initialize()
{
  m_imageQos.keep_last(10);
  m_imageQos.reliable();
  m_imageQos.durability_volatile();

  this->declare_parameter("prefix", "wrist_mounted_camera");
  m_prefix = this->get_parameter("prefix").as_string();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "image topic prefix: %s \nColor blob: %s", m_prefix.c_str(), m_color.c_str());

  float dilation_size=1.0;
  m_morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );

  m_color_srv = this->create_service<dex_ivr_interfaces::srv::BlobDimensions>("color_set_blob_dimensions", std::bind(&ColorBlobCentroid::color_set_blob_dimensions, this, _1, _2));
  m_processing_srv = this->create_service<std_srvs::srv::SetBool>("color_toggle_continuous", std::bind(&ColorBlobCentroid::toggle_continuous, this, _1, _2));

  m_depthImageSub.subscribe(this, "/" + m_prefix + "/aligned_depth_to_color/image_raw", m_imageQos.get_rmw_qos_profile());
  m_colorImageSub.subscribe(this,  "/" + m_prefix + "/color/image_raw", m_imageQos.get_rmw_qos_profile());
  m_colorInfoSub.subscribe(this,  "/" + m_prefix + "/color/camera_info", m_imageQos.get_rmw_qos_profile());

  m_timeSyncPtr = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                        sensor_msgs::msg::CameraInfo>>(m_colorImageSub, m_depthImageSub, m_colorInfoSub, 10);
  m_timeSyncPtr->registerCallback(std::bind(&ColorBlobCentroid::imageCallback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to process images on service request ");
}

void ColorBlobCentroid::color_set_blob_dimensions(const std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Request> request,
          std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Response>      response)
{
  std::string req = "Incoming Request - Aspect Ratio: " + std::to_string(request->aspect_ratio) + " Size: " + std::to_string(request->size);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), req.c_str());
  m_blobAspectRatio = request->aspect_ratio;
  m_blobARThreshold = request->aspect_ratio_threshold;
  m_blobSize = request->size;
  m_blobSizeThreshold = request->size_threshold;
  if (request->color != "")
    m_color = request->color;
  if (request->prefix != "")
    m_prefix = request->prefix;

  m_depthImageSub.subscribe(this, "/" + m_prefix + "/aligned_depth_to_color/image_raw", m_imageQos.get_rmw_qos_profile());
  m_colorImageSub.subscribe(this,  "/" + m_prefix + "/color/image_raw", m_imageQos.get_rmw_qos_profile());
  m_colorInfoSub.subscribe(this,  "/" + m_prefix + "/color/camera_info", m_imageQos.get_rmw_qos_profile());

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request color -> %s", request->color.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "image prefix -> %s", request->prefix.c_str());

  geometry_msgs::msg::Pose blobPos;
  processBlob(blobPos);
  response->centroid_pose = blobPos;

}

void ColorBlobCentroid::toggle_continuous(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Continuous output set to %d", request->data);
  m_continuousColor = request->data;
  response->success = true;
}

void ColorBlobCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                                         const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA)
{
  m_colorImage = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
  m_depthImage = cv::Mat(cv_bridge::toCvCopy(depthImMsgA)->image);
  m_imageInfo = *infoMsgA;
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

  if(m_continuousColor)
  {
    geometry_msgs::msg::Pose blobPos;
    processBlob(blobPos); //live processing for debugging
  }
}

void ColorBlobCentroid::processBlob(geometry_msgs::msg::Pose &blobPos)
{
  m_mask = 0;
  m_colorNames.createColorMask(m_colorImage, m_color, m_mask);

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

    //standardize the height and width regardless of orientation of strip or camera. height is the "longer portion of blob"
    double height = std::max(rotRect.size.height,rotRect.size.width);
    double width = std::min(rotRect.size.height,rotRect.size.width);
    double angle;
    angle = rotRect.angle;
    if (rotRect.size.height > rotRect.size.width)
      angle -= 90;
    if (angle < 0)
      angle+=180; //prevents 180 degreees inversion of gripper for rotation

    // if aspect ratio is set to something specific for testing, only process those nodes
    bool processContour = false;
    if ((m_blobAspectRatio == -1 &&  m_blobSize == -1 ) || (m_blobAspectRatio == 0 &&  m_blobSize == 0 ))
    {
      //eliminate noisey shadows
      if(height > MIN_OBJECT_SIZE && width > MIN_OBJECT_SIZE)
          processContour = true;
    }
    // if aspect ratio exists and is within threshold of what is expected.
    if (m_blobAspectRatio != -1 && (width / height) < (m_blobAspectRatio + m_blobARThreshold/2) &&  (width / height) > (m_blobAspectRatio - m_blobARThreshold/2))
    {
      // if size exists and meets min size requirement (close enough that it is not considered random noise)
      if(m_blobSize != -1 && height > MIN_OBJECT_SIZE && width > MIN_OBJECT_SIZE && height > (m_blobSize-m_blobSizeThreshold) && height < (m_blobSize + m_blobSizeThreshold))
      {
          processContour = true;
      }
    }


    // default behavior (Aspect/Size == -1) is to process all contours that appear
    if (processContour)
    {
      // calculate x,y coordinate of centroid
      cv::Moments moment = moments(contours[i]);
      cv::Point2f momentPt;
      if (moment.m00 != 0)
      { 

        momentPt = cv::Point2f(static_cast<float>(moment.m10 / moment.m00), static_cast<float>(moment.m01 / moment.m00));

        circle(m_colorImage, momentPt, 5, cv::Scalar(255, 255, 255), -1);
        std::string boxAR = "AR: " + std::to_string((width / height));
        std::string boxSize = "size: " + std::to_string(width) + " " + std::to_string(height) + " angle: " + std::to_string(angle);
        putText(m_colorImage, boxAR, cv::Point2f(momentPt.x - 25, momentPt.y - 25),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        putText(m_colorImage, boxSize, cv::Point2f(momentPt.x - 25, momentPt.y - 10),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        //Depth image stuff
        double depth = m_depthImage.at<float>(momentPt);
        std::string  depthPrint = "depth: " + std::to_string(depth) + " angle: " + std::to_string(rotRect.angle);
        putText(m_colorImage, depthPrint, cv::Point2f(momentPt.x - 25, momentPt.y + 20),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        double worldX = (momentPt.x-m_imageInfo.k.at(2)) * (depth/m_imageInfo.k.at(0)); // (x' - cx) * (depth/focal length x) --- where x' is image x in pixels and cx is center of image x from camera image
        double worldY = (momentPt.y-m_imageInfo.k.at(5)) * (depth/m_imageInfo.k.at(4)); // (y' - cy) * (depth/focal length y) --- where y' is image y in pixels and cy is center of image y from camera image
        std::string worldPos = "world X:" + std::to_string(worldX) + " world Y:" + std::to_string(worldY) + " world Z:" + std::to_string(depth);
        putText(m_colorImage, worldPos, cv::Point2f(momentPt.x - 25, momentPt.y + 35),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        //rotate frame according to long axis -- 90 degrees offset to make  the "vertical long axis" the 0-degree rotation. the gripper opens horizonally, should not need to rotate when object is vertically oriented as thats the correct orientation for grasp.
        tf2::Quaternion longAxis;
        longAxis.setRPY(0,0,((angle-90)*CV_PI/180));
        longAxis.normalize();

        //transform data to be published
        geometry_msgs::msg::Quaternion quat;
        quat = tf2::toMsg(longAxis);

        //set output for service call
        blobPos.position.x = worldX;
        blobPos.position.y = worldY;
        blobPos.position.z = depth;
        blobPos.orientation.x = longAxis.x();
        blobPos.orientation.y = longAxis.y();
        blobPos.orientation.z = longAxis.z();
        blobPos.orientation.w = longAxis.w();
        std::string output = "Object found at " + std::to_string(worldX) + ", " + std::to_string(worldY) + ", " + std::to_string(blobPos.position.z) + ", " + " angled (in degrees): " + std::to_string(angle) + "\n";
        
        if(!m_continuousColor) //dont clutter output terminal
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), output.c_str());

        //create and publish tf message
        geometry_msgs::msg::TransformStamped ts;
        ts.header = m_imageInfo.header;
        ts.child_frame_id= std::string("colorblob_xd");
        ts.transform.rotation = quat;
        ts.transform.translation.x = worldX;
        ts.transform.translation.y = worldY;
        ts.transform.translation.z = depth;
        m_tfBroadcasterPtr->sendTransform(ts);
      }
    }
  }
  // cv::imshow("colornames", m_mask);
  // cv::imshow("dilate -> eroded", eroded);
  // cv::imshow("depth image", m_depthImage);
  // cv::waitKey(0);

  // cv::imshow("final result", m_colorImage);
  // cv::waitKey(0); //set to 1 for coninuous output, set to 0 for single frame forever

}


int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "opencv : %s\n", CV_VERSION);
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ColorBlobCentroid>());
  rclcpp::shutdown();
  cv::destroyAllWindows();

  return 0;
}
