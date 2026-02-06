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

#include <opencv2/opencv.hpp>
#include <color_names/ColorNames.h>
#include "color_tools_msgs/msg/blob_request.hpp"
#include "color_tools_msgs/msg/blob_result.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace color_blob_centroid
{

/**
 * @brief Process color and depth images to find colored blobs
 *
 * @param colorImage BGR color image (will be annotated in place)
 * @param depthImage Depth image (CV_32FC1 in meters)
 * @param cameraInfo Camera intrinsic parameters
 * @param request Blob detection parameters
 * @return BlobResult containing pose and images
 */
color_tools_msgs::msg::BlobResult processBlobs(
    cv::Mat& colorImage,
    const cv::Mat& depthImage,
    const sensor_msgs::msg::CameraInfo& cameraInfo,
    const color_tools_msgs::msg::BlobRequest& request);

}  // namespace color_blob_centroid
