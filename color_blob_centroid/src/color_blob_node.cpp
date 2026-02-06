/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "color_blob_centroid/color_blob_node.hpp"
#include "color_blob_centroid/color_blob_centroid.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// save the image to a member and then process the image with what comes in the
// service call.
// for future streaming purposes, you would need to set up a multithreaded
// executor, put a wait in service, and then separate both subscribers into
// unique callback groups

ColorBlobCentroid::ColorBlobCentroid()
  : rclcpp::Node("color_blob_centroid")
  , m_blobSize(-1)
  , m_blobSizeThreshold(50)
  , m_blobAspectRatio(-1)
  , m_blobARThreshold(0.03)
  , m_imageQos(1)
  , m_continuousColor(false)
  , m_mockHardware(false)
  , m_showImage(false)
  , m_debugMode(false)
{
  initialize();
}

ColorBlobCentroid::~ColorBlobCentroid()
{
}

/****************
 * Initialize - Grab ROS params and start services/subscribers/image callback
 *****************/
void ColorBlobCentroid::initialize()
{
  m_imageQos.keep_last(10);
  m_imageQos.reliable();
  m_imageQos.durability_volatile();

  /** --- Ros Parameters --- */

  // continuous output of final transform
  this->declare_parameter("continuous_output", false);
  m_continuousColor = this->get_parameter("continuous_output").as_bool();
  RCLCPP_INFO(this->get_logger(), "Continuous Output set to %s", m_continuousColor ? "true" : "false");

  // image topic prefix - realsense spawn topics based on camera_name parameter
  this->declare_parameter("prefix", "wrist_mounted_camera");
  m_prefix = this->get_parameter("prefix").as_string();

  // color image topic
  this->declare_parameter("color_img_topic", "color/image_raw");
  m_color_topic = this->get_parameter("color_img_topic").as_string();

  // depth image topic
  this->declare_parameter("depth_img_topic", "aligned_depth_to_color/image_raw");
  m_depth_topic = this->get_parameter("depth_img_topic").as_string();

  // camera info topic
  this->declare_parameter("cam_info_topic", "color/camera_info");
  m_info_topic = this->get_parameter("cam_info_topic").as_string();

  // mock hardware - test operation without an image topic using a dummy point
  this->declare_parameter("mock_hardware", false);
  m_mockHardware = this->get_parameter("mock_hardware").as_bool();
  RCLCPP_INFO(this->get_logger(), "Mock Hardware set to %s", m_mockHardware ? "true !!! WARNING !!!" : "false");

  // show camera imagethis->declare_parameter("show_image", false);
  this->declare_parameter("show_image", false);
  m_showImage = this->get_parameter("show_image").as_bool();
  RCLCPP_INFO(this->get_logger(), "Show Image set to %s", m_showImage ? "true" : "false");

  // verbose debug mode that shows underlying color
  this->declare_parameter("debug", false);
  m_debugMode = this->get_parameter("debug").as_bool();
  RCLCPP_INFO(this->get_logger(), "Debug set to %s", m_debugMode ? "true" : "false");

  RCLCPP_INFO(this->get_logger(), "Initial settings:");
  RCLCPP_INFO(this->get_logger(), "Image topic prefix: %s", m_prefix.c_str());
  RCLCPP_INFO(this->get_logger(), "Color blob: %s", m_currentRequest.color.c_str());
  RCLCPP_INFO(this->get_logger(), "Color image topic: %s", ("/" + m_prefix + "/" + m_color_topic).c_str());
  RCLCPP_INFO(this->get_logger(), "Depth image topic: %s", ("/" + m_prefix + "/" + m_depth_topic).c_str());
  RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", ("/" + m_prefix + "/" + m_info_topic).c_str());

  // Construct publishers
  m_imagePub = this->create_publisher<sensor_msgs::msg::Image>("colorblob_image", 10);
  m_imageRawPub = this->create_publisher<sensor_msgs::msg::Image>("colorblob_image_raw", 10);
  m_maskPub = this->create_publisher<sensor_msgs::msg::Image>("colorblob_mask", 10);

  // Construct subscribers for image topics and synchronize them
  m_depthImageSub.subscribe(this, "/" + m_prefix + "/" + m_depth_topic, m_imageQos.get_rmw_qos_profile());
  m_colorImageSub.subscribe(this, "/" + m_prefix + "/" + m_color_topic, m_imageQos.get_rmw_qos_profile());
  m_colorInfoSub.subscribe(this, "/" + m_prefix + "/" + m_info_topic, m_imageQos.get_rmw_qos_profile());

  m_timeSyncPtr = std::make_shared<
      message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(
      m_colorImageSub, m_depthImageSub, m_colorInfoSub, 10);
  m_timeSyncPtr->registerCallback(std::bind(&ColorBlobCentroid::imageCallback, this, _1, _2, _3));

  // Construct service servers.
  m_processing_srv = this->create_service<std_srvs::srv::SetBool>(
      "color_toggle_continuous", std::bind(&ColorBlobCentroid::toggle_continuous, this, _1, _2));
  m_color_srv = this->create_service<color_tools_msgs::srv::BlobDimensions>(
      "color_set_blob_dimensions", std::bind(&ColorBlobCentroid::color_set_blob_dimensions, this, _1, _2));
  m_color_simple_srv = this->create_service<color_tools_msgs::srv::BlobCentroid>(
      "color_blob_find", std::bind(&ColorBlobCentroid::color_blob_find, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Ready to process images on service request");
}

/****************
 * Mock Hardware - Helper for when testing color blob on system without use of a
 *camera
 *****************/
bool ColorBlobCentroid::sendMockHardwareTransform(geometry_msgs::msg::PoseStamped& blobPos)
{
  if (!m_mockHardware)
    return false;

  RCLCPP_INFO(this->get_logger(), "!!!!! MOCK HARDWARE ENABLED - outputting fake response. !!!!!");
  rclcpp::Time now = this->get_clock()->now();
  blobPos.header.frame_id = m_prefix + "_color_optical_frame";
  blobPos.header.stamp = now;
  blobPos.pose.position.x = 0;
  blobPos.pose.position.y = 0;
  blobPos.pose.position.z = 0.5;
  blobPos.pose.orientation.x = 0;
  blobPos.pose.orientation.y = 0;
  blobPos.pose.orientation.z = 0;
  blobPos.pose.orientation.w = 1;
  RCLCPP_INFO(this->get_logger(), "MSG -> x:0 y:0 z:0.5 --- qx:0 qy:0 qz:0 qw:1");

  publishTransform(blobPos);
  return true;
}

void ColorBlobCentroid::publishTransform(const geometry_msgs::msg::PoseStamped& pose)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header = pose.header;
  ts.child_frame_id = "colorblob_xd";
  ts.transform.translation.x = pose.pose.position.x;
  ts.transform.translation.y = pose.pose.position.y;
  ts.transform.translation.z = pose.pose.position.z;
  ts.transform.rotation = pose.pose.orientation;
  m_tfBroadcasterPtr->sendTransform(ts);
}

void ColorBlobCentroid::color_blob_find(const std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Request> request,
                                        std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Response> response)
{
  geometry_msgs::msg::PoseStamped blobPos;
  if (sendMockHardwareTransform(blobPos))
  {
    response->result.centroid_pose = blobPos;
    return;
  }

  if (m_colorImage.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "ERROR - No image. Check that image topics exist and data is flowing.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Incoming Request - Min Blob Size: %.1f, Color: %s, Image prefix: %s",
              request->request.min_blob_size, request->request.color.c_str(), m_prefix.c_str());

  // Process using standalone function
  cv::Mat colorImageCopy = m_colorImage.clone();
  color_tools_msgs::msg::BlobResult result =
      color_blob_centroid::processBlobs(colorImageCopy, m_depthImage, m_imageInfo, request->request);

  // Publish images
  m_imagePub->publish(result.color_img);
  m_imageRawPub->publish(result.color_img_raw);
  m_maskPub->publish(result.mask);

  // Publish transform if pose is valid
  if (result.centroid_pose.header.frame_id != "")
  {
    publishTransform(result.centroid_pose);
    RCLCPP_INFO(this->get_logger(), "Object found at %.3f, %.3f, %.3f", result.centroid_pose.pose.position.x,
                result.centroid_pose.pose.position.y, result.centroid_pose.pose.position.z);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "FAILED to find object in image frame");
  }

  response->result = result;
}

void ColorBlobCentroid::color_set_blob_dimensions(
    const std::shared_ptr<color_tools_msgs::srv::BlobDimensions::Request> request,
    std::shared_ptr<color_tools_msgs::srv::BlobDimensions::Response> response)
{
  RCLCPP_INFO(this->get_logger(),
              "Incoming Request - AR: %.2f, AR Thresh: %.2f, Size: %.1f, Size Thresh: %.1f, Color: %s, Prefix: %s",
              request->aspect_ratio, request->aspect_ratio_threshold, request->size, request->size_threshold,
              request->color.c_str(), request->prefix.c_str());

  geometry_msgs::msg::PoseStamped blobPos;
  if (sendMockHardwareTransform(blobPos))
  {
    response->result.centroid_pose = blobPos;
    return;
  }

  if (m_colorImage.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "ERROR - No image. Check that image topics exist and data is flowing.");
    return;
  }

  // Update prefix if changed
  if (!request->prefix.empty() && request->prefix != m_prefix)
  {
    m_prefix = request->prefix;
    m_depthImageSub.subscribe(this, "/" + m_prefix + "/" + m_depth_topic, m_imageQos.get_rmw_qos_profile());
    m_colorImageSub.subscribe(this, "/" + m_prefix + "/" + m_color_topic, m_imageQos.get_rmw_qos_profile());
    m_colorInfoSub.subscribe(this, "/" + m_prefix + "/" + m_info_topic, m_imageQos.get_rmw_qos_profile());
  }

  // Build request message
  color_tools_msgs::msg::BlobRequest blobRequest;
  blobRequest.desired_blob = request->desired_blob;
  blobRequest.min_blob_size = request->size > 0 ? request->size - request->size_threshold : 10.0;
  blobRequest.color = request->color.empty() ? "red" : request->color;

  // Process using standalone function
  cv::Mat colorImageCopy = m_colorImage.clone();
  color_tools_msgs::msg::BlobResult result =
      color_blob_centroid::processBlobs(colorImageCopy, m_depthImage, m_imageInfo, blobRequest);

  // Publish images
  m_imagePub->publish(result.color_img);
  m_imageRawPub->publish(result.color_img_raw);
  m_maskPub->publish(result.mask);

  // Publish transform if pose is valid
  if (result.centroid_pose.header.frame_id != "")
  {
    publishTransform(result.centroid_pose);
    RCLCPP_INFO(this->get_logger(), "Object found at %.3f, %.3f, %.3f", result.centroid_pose.pose.position.x,
                result.centroid_pose.pose.position.y, result.centroid_pose.pose.position.z);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "FAILED to find object in image frame");
  }

  response->result = result;
}

void ColorBlobCentroid::toggle_continuous(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Continuous output set to %d", request->data);
  m_continuousColor = request->data;
  response->success = true;
}

void ColorBlobCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                                      const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA)
{
  m_colorImage = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);
  m_depthImage = cv::Mat(cv_bridge::toCvCopy(depthImMsgA)->image);
  m_imageInfo = *infoMsgA;

  // Normalize depth image
  if (m_depthImage.type() != CV_32FC1)
  {
    if (m_depthImage.type() == CV_16UC1)
    {
      m_depthImage.convertTo(m_depthImage, CV_32FC1, 0.001);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Depth image type must be CV_32FC1 or CV_16UC1");
    }
  }

  if (m_continuousColor)
  {
    cv::Mat colorImageCopy = m_colorImage.clone();
    color_tools_msgs::msg::BlobResult result =
        color_blob_centroid::processBlobs(colorImageCopy, m_depthImage, m_imageInfo, m_currentRequest);

    if (result.centroid_pose.header.frame_id != "")
    {
      publishTransform(result.centroid_pose);
    }

    if (m_showImage && !m_mockHardware)
      if (m_showImage && !m_mockHardware)
        if (m_showImage && !m_mockHardware)
        {
          cv::imshow("img", colorImageCopy);
          cv::waitKey(1);  // set to 1 for continuous output, set to 0 for single frame forever
        }
  }
}

int main(int argc, char* argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "opencv : %s\n", CV_VERSION);
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ColorBlobCentroid>());
  rclcpp::shutdown();
  cv::destroyAllWindows();

  return 0;
}
