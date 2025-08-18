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
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <color_names/ColorNames.h>
#include "dex_ivr_interfaces/srv/blob_centroid.hpp"
#include "dex_ivr_interfaces/srv/blob_dimensions.hpp"

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ColorBlobCentroid : public rclcpp::Node
{
public:
  ColorBlobCentroid();
  ~ColorBlobCentroid();
  static bool sortContour(std::vector<cv::Point> a, std::vector<cv::Point> b)
  {
    cv::Rect rectA = cv::boundingRect(a);
    cv::Rect rectB = cv::boundingRect(b);

    if (abs(rectA.y - rectB.y) <= 25)
      return (rectA.x < rectB.x);

    return (rectA.y < rectB.y);
  }

private:
  void initialize();

  /* Helpers */

  // makes fake transform position 0.5 units Z direction from camera optical frame
  bool sendMockHardwareTransform(geometry_msgs::msg::PoseStamped& blobPos);

  // calculates final realworld coordinates of specific contour, writes data to image
  void processContour(geometry_msgs::msg::PoseStamped& blobPos, cv::Point2f momentPt, cv::RotatedRect rotRect);

  // verify that a contour is within thresholds set by services
  bool checkValidContour(cv::RotatedRect rotRect);

  // using computed blob metrics, output for service using first two parameters, and publish transform of
  // blob location
  void outputContour(geometry_msgs::msg::PoseStamped& blobPos, double worldX, double worldY, double depth, double angle);

  // given a header, cv::Mat, and image encoding, create a ROS image
  void convertCVImageToROS(cv::Mat& input, const char encoding[], sensor_msgs::msg::Image& output);

  /* Services */

  void color_blob_find(const std::shared_ptr<dex_ivr_interfaces::srv::BlobCentroid::Request> request,
                       std::shared_ptr<dex_ivr_interfaces::srv::BlobCentroid::Response> response);
  void color_set_blob_dimensions(const std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Request> request,
                                 std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Response> response);
  void toggle_continuous(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /* Core Processing */

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA);

  // iterates through all color blobs in image and filters them with openCV & the thresholds specified by service
  void processBlobs(geometry_msgs::msg::PoseStamped& blobPos);

  // Blob filtering parameters to maintain between service calls
  double m_minBlobSize;
  double m_blobSize;
  double m_blobSizeThreshold;
  double m_blobAspectRatio;
  double m_blobARThreshold;
  std::string m_prefix;
  std::string m_depth_topic;
  std::string m_color_topic;
  std::string m_info_topic;

  // ROS stuff
  rclcpp::QoS m_imageQos;
  rclcpp::Service<dex_ivr_interfaces::srv::BlobDimensions>::SharedPtr
      m_color_srv;  // configures the parameters of the color blob detection:
                    // aspect ratio, size, color
  rclcpp::Service<dex_ivr_interfaces::srv::BlobCentroid>::SharedPtr
      m_color_simple_srv;                                               // configures the parameters of the color blob
                                                                        // detection: min blob size and color
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_processing_srv;  // turn on/off continuous image processing

  // NOTE: for Emma/future parties wondering why we bother publishing a one-time
  // image out on topics. They are published so that the data is captured in
  // ROSBAG because service calls and results are not captured So the exact
  // frame used for colorblob would be lost on replay, this allows you to
  // diagnose/recompute/reinterpret with correct images.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imagePub;  // markup image (what is shown to user with blobs
                                                                     // circled in green)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      m_imageRawPub;  // raw frame used for color blob detection with no markup
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_maskPub;  // mask of color, **NOTE** closed contours of color
                                                                    // will FILL the contour!! A ring of color will
                                                                    // output a filled circle of a mask!
  message_filters::Subscriber<sensor_msgs::msg::Image> m_depthImageSub;
  message_filters::Subscriber<sensor_msgs::msg::Image> m_colorImageSub;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_colorInfoSub;
  std::shared_ptr<
      message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>
      m_timeSyncPtr;

  // Image checkpoints
  cv::Mat m_mask;
  ColorNames m_colorNames;
  cv::Mat m_colorImage;
  cv::Mat m_colorImageRaw;
  cv::Mat m_depthImage;
  cv::Mat m_morphology;

  // Service call thresholds
  std::string m_color;
  bool m_continuousColor;  // flag for continuous processing of color
  bool m_mockHardware;
  bool m_showImage;
  bool m_debugMode;
  uint m_desiredBlob;
  uint m_blobNum;
  sensor_msgs::msg::CameraInfo m_imageInfo;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcasterPtr =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};
