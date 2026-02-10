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

#include <color_blob_centroid/color_blob_centroid.hpp>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

// Support humble and jazzy
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

TEST(ProcessBlobsTest, DetectsRedBlob)
{
  // Configure a dummy test camera
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = "test_camera";
  camera_info.k = { 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0 };

  // Generate a black image with a 50 pixel red circle in the middle.
  // Depth image universally 0.5 m away.
  cv::Mat color_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::circle(color_image, cv::Point(320, 240), 50, cv::Scalar(0, 0, 255), -1);
  cv::Mat depth_image(480, 640, CV_32FC1, cv::Scalar(0.5));

  color_blob_centroid::BlobRequest request;
  cv_bridge::CvImage(camera_info.header, sensor_msgs::image_encodings::BGR8, color_image).toImageMsg(request.color_img);
  cv_bridge::CvImage(camera_info.header, sensor_msgs::image_encodings::TYPE_32FC1, depth_image)
      .toImageMsg(request.depth_img);
  request.camera_info = camera_info;

  const auto result = color_blob_centroid::processBlobs(request);

  // Verify blob was found
  EXPECT_TRUE(result.success);
  EXPECT_NE(result.centroid_pose.header.frame_id, "");
  EXPECT_NEAR(result.centroid_pose.pose.position.z, 0.5, 0.01);
  EXPECT_NEAR(result.centroid_pose.pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(result.centroid_pose.pose.position.y, 0.0, 0.01);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
