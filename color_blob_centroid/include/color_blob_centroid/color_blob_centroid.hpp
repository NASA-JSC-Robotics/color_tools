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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <color_names/ColorNames.h>
#include <opencv2/opencv.hpp>

namespace color_blob_centroid
{

/**
 * @brief Container for the results of a blob process result.
 */
struct BlobResult
{
  /// @brief Whether or not processing was successful
  bool success;

  /// @brief The pose of the requested blob centroid
  geometry_msgs::msg::PoseStamped centroid_pose;

  /// @brief Annoted color image complete with mask information
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
 * @param colorImage BGR color image (will be annotated in place)
 * @param depthImage Depth image
 * @param cameraInfo Camera intrinsic parameters
 * @param request Blob detection parameters
 * @param blob_color Target blob color, defaults to "red"
 * @param min_blob_size Minimum pixels of the blob, defaults to 10.0
 * @param desired_blob Index of the requested blob in the detections list
 * @return BlobResult containing pose and images
 */
BlobResult processBlobs(cv::Mat& colorImage, const cv::Mat& depthImage,
                                             const sensor_msgs::msg::CameraInfo& cameraInfo,
                                             const std::string blob_color = std::string("red"),
                                             const double min_blob_size = 10.0,
                                             const uint8_t desired_blob = 0);

}  // namespace color_blob_centroid
