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

#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <color_names/ColorNames.h>
#include "color_tools_msgs/msg/blob_request.hpp"
#include "color_tools_msgs/msg/blob_result.hpp"
#include "color_tools_msgs/srv/blob_centroid.hpp"
#include "color_tools_msgs/srv/blob_dimensions.hpp"

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Support humble and jazzy+
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

class ColorBlobCentroid : public rclcpp::Node
{
public:
  ColorBlobCentroid();
  ~ColorBlobCentroid();

private:
  void initialize();

  /* Helpers */
  geometry_msgs::msg::PoseStamped sendMockHardwareTransform();
  void publishTransform(const geometry_msgs::msg::PoseStamped& pose);

  /* Services */
  void color_blob_find(const std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Request> request,
                       std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Response> response);
  void color_set_blob_dimensions(const std::shared_ptr<color_tools_msgs::srv::BlobDimensions::Request> request,
                                 std::shared_ptr<color_tools_msgs::srv::BlobDimensions::Response> response);
  void toggle_continuous(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /* Core Processing */
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA);

  // Current request parameters (updated by services)
  color_tools_msgs::msg::BlobRequest m_currentRequest;

  // Extended parameters for BlobDimensions service
  double m_blobSize;
  double m_blobSizeThreshold;
  double m_blobAspectRatio;
  double m_blobARThreshold;
  double m_staleMessageTimeout;

  std::string m_prefix;
  std::string m_depth_topic;
  std::string m_color_topic;
  std::string m_info_topic;

  // ROS stuff
  rclcpp::QoS m_imageQos;
  rclcpp::Service<color_tools_msgs::srv::BlobDimensions>::SharedPtr m_color_srv;
  rclcpp::Service<color_tools_msgs::srv::BlobCentroid>::SharedPtr m_color_simple_srv;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_processing_srv;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imagePub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imageRawPub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_maskPub;

  message_filters::Subscriber<sensor_msgs::msg::Image> m_depthImageSub;
  message_filters::Subscriber<sensor_msgs::msg::Image> m_colorImageSub;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_colorInfoSub;
  std::shared_ptr<
      message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>
      m_timeSyncPtr;

  // Image data
  cv::Mat m_colorImage;
  cv::Mat m_depthImage;
  sensor_msgs::msg::CameraInfo m_imageInfo;

  // Flags
  bool m_continuousColor;
  bool m_mockHardware;
  bool m_showImage;
  bool m_debugMode;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcasterPtr =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};
