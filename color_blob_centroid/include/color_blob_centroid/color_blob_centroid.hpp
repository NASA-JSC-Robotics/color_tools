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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <color_names/ColorNames.h>
#include <opencv2/opencv.hpp>

namespace color_blob_centroid
{

/**
 * @brief Container for a blob process request.
 */
struct BlobRequest
{
  /// @brief The color image to search
  sensor_msgs::msg::Image color_img;

  /// @brief The depth image to search
  sensor_msgs::msg::Image depth_img;

  /// @brief Synced camera info for the depth/color images
  sensor_msgs::msg::CameraInfo camera_info;

  /// @brief Target blob color, defaults to "red"
  std::string blob_color = std::string("red");

  /// @brief Minimum pixels of the blob, defaults to 10.0
  double min_blob_size = 10.0;

  /// @brief Index of the requested blob in the detections list
  uint8_t desired_blob = 0;
};

/**
 * @brief Container for a blob process result.
 */
struct BlobResult
{
  /// @brief Whether or not processing was successful
  bool success;

  /// @brief Optional message in the event of failure
  std::string err_msg;

  /// @brief The pose of the requested blob centroid
  geometry_msgs::msg::PoseStamped centroid_pose;

  /// @brief Annotated color image complete with mask information
  sensor_msgs::msg::Image color_img;

  /// @brief Mask of the detected color blobs
  sensor_msgs::msg::Image mask;

  /// @brief Original color image in ROS message format
  sensor_msgs::msg::Image color_img_raw;

  /// @brief Original depth image in ROS message format
  sensor_msgs::msg::Image depth_img;
};

/**
 * @brief Process color and depth images to find colored blobs
 *
 * @param request BlobRequest with required information
 * @return BlobResult containing pose and images
 */
BlobResult processBlobs(const BlobRequest& request);

}  // namespace color_blob_centroid
