#include "color_blob_centroid/ColorBlobCentroid.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


// save the image to a member and then process the image with what comes in the service call.
//for future streaming purposes, you would need to set up a multithreaded executor, put a wait in service, and then separate both subscribers into unique callback groups

ColorBlobCentroid::ColorBlobCentroid()
  : rclcpp::Node("color_blob_centroid")
  , m_minBlobSize(10)
  , m_blobSize(-1)
  , m_blobSizeThreshold(50)
  , m_blobAspectRatio(-1)
  , m_blobARThreshold(0.03)
  , m_imageQos(1)
  , m_color("red")
  , m_continuousColor(false)
  , m_mockHardware(false)
  , m_showImage(false)
  , m_desiredBlob(0)
{
  initialize();
}

ColorBlobCentroid::~ColorBlobCentroid(){}


/****************
 * Initialize - Grab ROS params and start services/subscribers/image callback
*****************/
void ColorBlobCentroid::initialize()
{
  m_imageQos.keep_last(10);
  m_imageQos.reliable();
  m_imageQos.durability_volatile();

  //  --- Ros Parameters ---
  //continuous output of final transform
  this->declare_parameter("continuous_output", false);
  m_continuousColor = this->get_parameter("continuous_output").as_bool();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Continuous Output set to %s", m_continuousColor?"true":"false");
  //image topic prefix - realsense spawn topics based on camera_name parameter
  this->declare_parameter("prefix", "wrist_mounted_camera");
  m_prefix = this->get_parameter("prefix").as_string();
  //color image topic
  this->declare_parameter("color_img_topic", "color/image_raw");
  m_color_topic = this->get_parameter("color_img_topic").as_string();
  //depth image topic
  this->declare_parameter("depth_img_topic", "aligned_depth_to_color/image_raw");
  m_depth_topic = this->get_parameter("depth_img_topic").as_string();
  //camera info topic
  this->declare_parameter("cam_info_topic", "color/camera_info");
  m_info_topic = this->get_parameter("cam_info_topic").as_string();
  //mock hardware - test operation without an image topic using a dummy point
  this->declare_parameter("mock_hardware", false);
  m_mockHardware = this->get_parameter("mock_hardware").as_bool();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mock Hardware set to %s", m_mockHardware?"true !!! WARNING !!!":"false");
  //show camera imagethis->declare_parameter("show_image", false);
  this->declare_parameter("show_image", false);
  m_showImage = this->get_parameter("show_image").as_bool();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Show Image set to %s", m_showImage?"true":"false");
    //verbose debug mode that shows underlying color
  this->declare_parameter("debug", false);
  m_debugMode = this->get_parameter("debug").as_bool();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Debug, set to %s", m_debugMode?"true":"false");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial settings:");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image topic prefix: %s", m_prefix.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Color blob: %s", m_color.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Color image topic: %s", ("/" + m_prefix + "/" + m_color_topic).c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth image topic: %s", ("/" + m_prefix + "/" + m_depth_topic).c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info topic: %s", ("/" + m_prefix + "/" + m_info_topic).c_str());

  //  --- Image Processing Parameters --
  float dilation_size=1.0;
  m_morphology = getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                      cv::Point( dilation_size, dilation_size ) );

  m_processing_srv = this->create_service<std_srvs::srv::SetBool>("color_toggle_continuous", std::bind(&ColorBlobCentroid::toggle_continuous, this, _1, _2));

  m_imagePub = this->create_publisher<sensor_msgs::msg::Image>("colorblob_image", 10);
  m_imageRawPub = this->create_publisher<sensor_msgs::msg::Image>("colorblob_image_raw", 10);
  m_maskPub = this->create_publisher<sensor_msgs::msg::Image>("colorblob_mask", 10);

  m_depthImageSub.subscribe(this, "/" + m_prefix + "/" + m_depth_topic, m_imageQos.get_rmw_qos_profile());
  m_colorImageSub.subscribe(this,  "/" + m_prefix + "/" + m_color_topic, m_imageQos.get_rmw_qos_profile());
  m_colorInfoSub.subscribe(this,  "/" + m_prefix + "/" + m_info_topic, m_imageQos.get_rmw_qos_profile());

  m_timeSyncPtr = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                        sensor_msgs::msg::CameraInfo>>(m_colorImageSub, m_depthImageSub, m_colorInfoSub, 10);
  m_timeSyncPtr->registerCallback(std::bind(&ColorBlobCentroid::imageCallback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));

  // Construct service servers.
  advertiseServices();
}

/****************
 * Mock Hardware - Helper for when testing color blob on system without use of a camera
*****************/
bool ColorBlobCentroid::sendMockHardwareTransform(geometry_msgs::msg::PoseStamped &blobPos)
{
    if(m_mockHardware)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "!!!!! MOCK HARDWARE ENABLED - outputting fake response. !!!!!");
        rclcpp::Time now = this->get_clock()->now();
        blobPos.header.frame_id = std::string(m_prefix + "_color_optical_frame");
        blobPos.header.stamp = now;
        blobPos.pose.position.x = 0;
        blobPos.pose.position.y = 0;
        blobPos.pose.position.z = 0.5;
        blobPos.pose.orientation.x = 0;
        blobPos.pose.orientation.y = 0;
        blobPos.pose.orientation.z = 0;
        blobPos.pose.orientation.w = 1;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MSG -> x:0 y:0 z:0.5 --- qx:0 qy:0 qz:0 qw:1 \n\n");

        //create and publish tf message
        geometry_msgs::msg::TransformStamped ts;
        ts.header.frame_id = std::string(m_prefix + "_color_optical_frame");
        ts.header.stamp = now;
        ts.child_frame_id= std::string("colorblob_xd");
        ts.transform.rotation.y = blobPos.pose.orientation.y;
        ts.transform.rotation.z = blobPos.pose.orientation.z;
        ts.transform.rotation.w = blobPos.pose.orientation.w;
        ts.transform.rotation.x = blobPos.pose.orientation.x;
        ts.transform.translation.x = blobPos.pose.position.x;
        ts.transform.translation.y = blobPos.pose.position.y;
        ts.transform.translation.z = blobPos.pose.position.z;
        m_tfBroadcasterPtr->sendTransform(ts);

        return true;
      }
      return false;
}

/****************
 * Convert cv Image to ROS Image - Given ros header, image encoding, opencv matrix, convert to sensor_msgs::Image
*****************/
void ColorBlobCentroid::convertCVImageToROS(cv::Mat &input, const char encoding[], sensor_msgs::msg::Image &output)
{
  //convert cv images to ROS images
  //img output
  cv_bridge::CvImage img_bridge;
  std_msgs::msg::Header header;
  img_bridge = cv_bridge::CvImage(header, encoding, input);
  img_bridge.toImageMsg(output); // from cv_bridge to sensor_msgs::Image
}

/****************
 * Output Contour- given a contours caluclations, prepare it for output, and publish transform
*****************/
void ColorBlobCentroid::outputContour(geometry_msgs::msg::PoseStamped &blobPos, double worldX, double worldY, double depth, double angle)
{
  //rotate frame according to long axis -- 90 degrees offset to make  the "vertical long axis" the 0-degree rotation. the gripper opens horizonally, should not need to rotate when object is vertically oriented as thats the correct orientation for grasp.
  tf2::Quaternion longAxis;
  longAxis.setRPY(0,0,((angle-90)*CV_PI/180));
  longAxis.normalize();

  //transform data to be published
  geometry_msgs::msg::Quaternion quat;
  quat = tf2::toMsg(longAxis);

  rclcpp::Time now = this->get_clock()->now();
  //set output for service call
  blobPos.header.frame_id = std::string(m_imageInfo.header.frame_id);
  blobPos.header.stamp = now;
  blobPos.pose.position.x = worldX;
  blobPos.pose.position.y = worldY;
  blobPos.pose.position.z = depth;
  blobPos.pose.orientation.x = longAxis.x();
  blobPos.pose.orientation.y = longAxis.y();
  blobPos.pose.orientation.z = longAxis.z();
  blobPos.pose.orientation.w = longAxis.w();

  //create and publish tf message
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = std::string(m_imageInfo.header.frame_id);

  ts.header.stamp = now;
  ts.child_frame_id= std::string("colorblob_xd");
  ts.transform.rotation.x = blobPos.pose.orientation.x;
  ts.transform.rotation.y = blobPos.pose.orientation.y;
  ts.transform.rotation.z = blobPos.pose.orientation.z;
  ts.transform.rotation.w = blobPos.pose.orientation.w;
  ts.transform.translation.x = blobPos.pose.position.x;
  ts.transform.translation.y = blobPos.pose.position.y;
  ts.transform.translation.z = blobPos.pose.position.z;
  m_tfBroadcasterPtr->sendTransform(ts);
}

/****************
 * Process Contour- uses moment of a contour for image markup and output
*****************/
void ColorBlobCentroid::processContour(geometry_msgs::msg::PoseStamped &blobPos, cv::Point2f momentPt, cv::RotatedRect rotRect)
{
  //standardize the height and width regardless of orientation of strip or camera. height is the "longer portion of blob"
  double height = std::max(rotRect.size.height,rotRect.size.width);
  double width = std::min(rotRect.size.height,rotRect.size.width);
  double angle;
  angle = rotRect.angle;
  if (rotRect.size.height > rotRect.size.width)
    angle -= 90;
  if (angle < 0)
    angle+=180; //prevents 180 degreees inversion of gripper for rotation

  //set blob text color based on desired blob
  bool doOutput = (m_desiredBlob == m_blobNum);
  cv::Scalar color;
  if(doOutput)
    color = cv::Scalar(70,255,70);
  else
    color = cv::Scalar(255,255,255);


  circle(m_colorImage, momentPt, 5, color, -1);
  putText(m_colorImage, std::to_string(m_blobNum), cv::Point2f(momentPt.x - 10, momentPt.y - 25),cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
  m_blobNum++;
  //Depth image stuff
  double depth = m_depthImage.at<float>(momentPt);
  double worldX = (momentPt.x-m_imageInfo.k.at(2)) * (depth/m_imageInfo.k.at(0)); // (x' - cx) * (depth/focal length x) --- where x' is image x in pixels and cx is center of image x from camera image
  double worldY = (momentPt.y-m_imageInfo.k.at(5)) * (depth/m_imageInfo.k.at(4)); // (y' - cy) * (depth/focal length y) --- where y' is image y in pixels and cy is center of image y from camera image
  if (m_debugMode)
  {
    std::string boxAR = "AR: " + std::to_string((width / height)) + " angle: " + std::to_string(angle);
    std::string boxSize = "W: " + std::to_string(int(width)) + " H:" + std::to_string(int(height));
    //putText(m_colorImage, boxAR, cv::Point2f(momentPt.x - 50, momentPt.y - 10),cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    putText(m_colorImage, boxSize, cv::Point2f(momentPt.x - 50, momentPt.y + 20),cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

    std::string worldPos = "world X:" + std::to_string(worldX) + " world Y:" + std::to_string(worldY) + " world Z:" + std::to_string(depth);
    //putText(m_colorImage, worldPos, cv::Point2f(momentPt.x - 50, momentPt.y + 40),cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
  }

  if (doOutput && depth == 0.0) //if the depth camera is too close to the object, dont publish transforms
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR - Depth Camera too close/far from object.");
      return;
    }

  //if this is the chosen blob (default blob 0), output blob
  if(doOutput)
    ColorBlobCentroid::outputContour(blobPos, worldX, worldY, depth, angle);

}

bool ColorBlobCentroid::checkValidContour(cv::RotatedRect rotRect)
{
  //standardize the height and width regardless of orientation of strip or camera. height is the "longer portion of blob"
  double height = std::max(rotRect.size.height,rotRect.size.width);
  double width = std::min(rotRect.size.height,rotRect.size.width);

  //Determine if this contour is within thresholds specified by service
  // if aspect ratio is set to something specific for testing, only process those nodes
  // (Aspect/Size == -1) to process all contours that appear
  bool processContour = false;
  if ((m_blobAspectRatio == -1 &&  m_blobSize == -1 ) || (m_blobAspectRatio == 0 &&  m_blobSize == 0 ))
  {
    //eliminate noisey shadows
    if(height > m_minBlobSize && width > m_minBlobSize)
        processContour = true;
  }
  // if aspect ratio exists and is within threshold of what is expected.
  if (m_blobAspectRatio != -1 && (width / height) < (m_blobAspectRatio + m_blobARThreshold/2) &&  (width / height) > (m_blobAspectRatio - m_blobARThreshold/2))
  {
    // if size exists and meets min size requirement (close enough that it is not considered random noise)
    if(m_blobSize != -1 && height > m_minBlobSize && width > m_minBlobSize && height > (m_blobSize-m_blobSizeThreshold) && height < (m_blobSize + m_blobSizeThreshold))
    {
        processContour = true;
    }
  }
  return processContour;
}
/****************
 * Color Blob Find Service - minimalist service call to get color, only uses blob size and blob color
*****************/
void ColorBlobCentroid::color_blob_find(const std::shared_ptr<dex_ivr_interfaces::srv::BlobCentroid::Request> request,
          std::shared_ptr<dex_ivr_interfaces::srv::BlobCentroid::Response>      response)
{
  //handle mock hardware
  geometry_msgs::msg::PoseStamped blobPos;
  if (sendMockHardwareTransform(blobPos))
  {
    response->centroid_pose = blobPos;
    return;
  }
  if (m_colorImage.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR - No image. Check that image topics exist and data is flowing.");
    return;
  }

  //if not mock_hardware actually proces the visual node
  m_desiredBlob = request->desired_blob;
  m_minBlobSize = request->min_blob_size;
  if (request->color != "") //change color if given new one
    m_color = request->color;

  std::string req = "Incoming Request -\n\nMin Blob Size: " + std::to_string(m_minBlobSize) + "\nColor: " + m_color + "\nImage prefix: " + m_prefix;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), req.c_str());

  processBlobs(blobPos);
  response->centroid_pose = blobPos;

  //final image conversion for output
  sensor_msgs::msg::Image blobImg, maskImg, rawImg, depthImg;
  convertCVImageToROS(m_colorImage, sensor_msgs::image_encodings::BGR8, blobImg);
  convertCVImageToROS(m_colorImageRaw, sensor_msgs::image_encodings::BGR8, rawImg);
  convertCVImageToROS(m_mask, sensor_msgs::image_encodings::MONO8, maskImg);
  convertCVImageToROS(m_depthImage, sensor_msgs::image_encodings::TYPE_32FC1, depthImg);
  blobImg.header = blobPos.header;
  rawImg.header = blobPos.header;
  maskImg.header = blobPos.header;
  depthImg.header = blobPos.header;
  response->color_img = blobImg;
  response->color_img_raw = rawImg;
  response->mask = maskImg;
  response->depth_img = depthImg;
  response->cam_info = m_imageInfo;
  m_imagePub->publish(blobImg);
  m_imageRawPub->publish(rawImg);
  m_maskPub->publish(maskImg);

  if (blobPos.header.frame_id != "")
  {
    std::string output = "Object found at " + std::to_string(blobPos.pose.position.x) + ", " + std::to_string(blobPos.pose.position.y) + ", " + std::to_string(blobPos.pose.position.z) + ", " + " angled: " + std::to_string(blobPos.pose.orientation.x) + ", "+ std::to_string(blobPos.pose.orientation.y) + ", "+ std::to_string(blobPos.pose.orientation.z) + ", "+ std::to_string(blobPos.pose.orientation.w) + "\n\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), output.c_str());
  }
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FAILED to find object in image frame");
}

/****************
 * Color Blob Set Blob Dimenstions Service - advanced service call to get color, has many settable parameters
*****************/
void ColorBlobCentroid::color_set_blob_dimensions(const std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Request> request,
          std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Response>      response)
{
  std::string req = "Incoming Request -\n Aspect Ratio: " + std::to_string(request->aspect_ratio) +  "\n Aspect Thresh: " + std::to_string(request->aspect_ratio_threshold)
                     + "\n Size: " + std::to_string(request->size) + "\n Size Thresh: " + std::to_string(request->size_threshold)
                     + "\n Color: " + request->color.c_str() + "\n Image prefix: " + request->prefix.c_str();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), req.c_str());
  //handle mock hardware
  geometry_msgs::msg::PoseStamped blobPos;
  if (sendMockHardwareTransform(blobPos))
  {
    response->centroid_pose = blobPos;
    return;
  }
  if (m_colorImage.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR - No image. Check that image topics exist and data is flowing.");
    return;
  }
  //if not mock_hardware actually proces the visual node
  m_blobAspectRatio = request->aspect_ratio;
  m_blobARThreshold = request->aspect_ratio_threshold;
  m_blobSize = request->size;
  m_blobSizeThreshold = request->size_threshold;
  m_desiredBlob = request->desired_blob;
  if (request->color != "") //change color if given new one
    m_color = request->color;
  if (request->prefix != "") //if new prefix given, change image topic subscribers
  {
    m_prefix = request->prefix;
    m_depthImageSub.subscribe(this, "/" + m_prefix + "/" + m_depth_topic, m_imageQos.get_rmw_qos_profile());
    m_colorImageSub.subscribe(this,  "/" + m_prefix + "/" + m_color_topic, m_imageQos.get_rmw_qos_profile());
    m_colorInfoSub.subscribe(this,  "/" + m_prefix + "/" + m_info_topic, m_imageQos.get_rmw_qos_profile());
  }

  processBlobs(blobPos);
  response->centroid_pose = blobPos;

  //final image conversion for output
  sensor_msgs::msg::Image blobImg, maskImg, rawImg, depthImg;
  convertCVImageToROS(m_colorImage, sensor_msgs::image_encodings::BGR8, blobImg);
  convertCVImageToROS(m_colorImageRaw, sensor_msgs::image_encodings::BGR8, rawImg);
  convertCVImageToROS(m_mask, sensor_msgs::image_encodings::MONO8, maskImg);
  convertCVImageToROS(m_depthImage, sensor_msgs::image_encodings::TYPE_32FC1, depthImg);
  blobImg.header = blobPos.header;
  rawImg.header = blobPos.header;
  maskImg.header = blobPos.header;
  depthImg.header = blobPos.header;
  response->color_img = blobImg;
  response->color_img_raw = rawImg;
  response->mask = maskImg;
  response->depth_img = depthImg;
  response->cam_info = m_imageInfo;
  m_imagePub->publish(blobImg);
  m_imageRawPub->publish(rawImg);
  m_maskPub->publish(maskImg);

  if (blobPos.header.frame_id != "")
  {
    std::string output = "Object found at " + std::to_string(blobPos.pose.position.x) + ", " + std::to_string(blobPos.pose.position.y) + ", " + std::to_string(blobPos.pose.position.z) + ", " + " angled: " + std::to_string(blobPos.pose.orientation.x) + ", "+ std::to_string(blobPos.pose.orientation.y) + ", "+ std::to_string(blobPos.pose.orientation.z) + ", "+ std::to_string(blobPos.pose.orientation.w) + "\n\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), output.c_str());
  }
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "FAILED to find object in image frame");
}

/****************
 * Toggle Continuous Service - toggle for setting whether you want continuous image processing
*****************/
void ColorBlobCentroid::toggle_continuous(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Continuous output set to %d", request->data);
  m_continuousColor = request->data;
  response->success = true;
}

/****************
 * Image Callback - Handles incoming image, passes to processing
*****************/
void ColorBlobCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                                         const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA)
{
  m_colorImage = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
  m_colorImageRaw = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
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
    geometry_msgs::msg::PoseStamped blobPos;
    processBlobs(blobPos); //live processing for debugging
  }
}

/****************
 * Process Blob - OpenCV & colorblob image processing
*****************/
void ColorBlobCentroid::processBlobs(geometry_msgs::msg::PoseStamped &blobPos)
{
  if (sendMockHardwareTransform(blobPos))
    return;

  if (m_colorImage.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR - No image. Check that image topics exist and data is flowing.");
    return;
  }

  m_blobNum = 0;
  m_mask = 0;
  m_colorNames.createColorMask(m_colorImage, m_color, m_mask);

  cv::Mat dilated, eroded;

  //erode first to delete smol noise outside obect of interest, dilate to restore object of interest to proper size
  erode(m_mask, eroded, m_morphology);
  dilate(eroded, dilated, m_morphology);

  //dilates to fill in noise inside object of interest and make it solid. erode to restore object to proper size
  dilate(dilated, dilated, m_morphology);
  erode(dilated, eroded, m_morphology);
  m_mask.setTo(cv::Scalar(0,0,0));

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(eroded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::sort(contours.begin(), contours.end(), ColorBlobCentroid::sortContour); //sort contours

  for (size_t i =0; i < contours.size(); i++)
  {
    // draw Rotated Rect
    cv::RotatedRect rotRect = minAreaRect(contours[i]);
    if (ColorBlobCentroid::checkValidContour(rotRect))
    {
      if (m_desiredBlob == m_blobNum)
      {
        drawContours(m_colorImage, std::vector<std::vector<cv::Point> >(1,contours[i]), -1, cv::Scalar(50, 200, 50), 4, cv::LINE_8);
        drawContours(m_mask, contours, i, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);
      }
      else
      {
        drawContours(m_colorImage, std::vector<std::vector<cv::Point> >(1,contours[i]), -1, cv::Scalar(0, 255, 255), 1, cv::LINE_8);
      }
      // calculate x,y coordinate of centroid
      cv::Moments moment = moments(contours[i]);
      if (moment.m00 != 0)
      {
        ColorBlobCentroid::processContour(blobPos, cv::Point2f(static_cast<float>(moment.m10 / moment.m00), static_cast<float>(moment.m01 / moment.m00)),rotRect);
      }
    }
  }

  if(m_showImage && !m_mockHardware )
  {
    cv::imshow("img", m_colorImage);
    if (m_debugMode)
    {
      cv::imshow("color segmentation", eroded);
    }
  }
    cv::waitKey(1); //set to 1 for coninuous output, set to 0 for single frame forever
}

void ColorBlobCentroid::advertiseServices()
{
  m_color_srv = this->create_service<dex_ivr_interfaces::srv::BlobDimensions>("color_set_blob_dimensions", std::bind(&ColorBlobCentroid::color_set_blob_dimensions, this, _1, _2));
  m_color_simple_srv = this->create_service<dex_ivr_interfaces::srv::BlobCentroid>("color_blob_find", std::bind(&ColorBlobCentroid::color_blob_find, this, _1, _2));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to process images on service request ");
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
