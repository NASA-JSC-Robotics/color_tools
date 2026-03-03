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

#include "color_blob_centroid/color_blob_centroid.hpp"

// Support humble and jazzy
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace color_blob_centroid
{

namespace
{

// Sort contours by position (top-to-bottom, left-to-right)
bool sortContour(const std::vector<cv::Point>& a, const std::vector<cv::Point>& b)
{
  cv::Rect rectA = cv::boundingRect(a);
  cv::Rect rectB = cv::boundingRect(b);

  if (abs(rectA.y - rectB.y) <= 25)
    return (rectA.x < rectB.x);

  return (rectA.y < rectB.y);
}

// Check if contour meets size requirements
bool checkValidContour(const cv::RotatedRect& rotRect, double minBlobSize)
{
  double height = std::max(rotRect.size.height, rotRect.size.width);
  double width = std::min(rotRect.size.height, rotRect.size.width);
  return (height > minBlobSize && width > minBlobSize);
}

}  // anonymous namespace

BlobResult processBlobs(const BlobRequest& request)
{
  BlobResult result;

  const auto colorImageRaw = cv::Mat(cv_bridge::toCvCopy(request.color_img, "bgr8")->image);
  auto depthImageRaw = cv::Mat(cv_bridge::toCvCopy(request.depth_img)->image);
  const auto cameraInfo = request.camera_info;

  if (colorImageRaw.empty())
  {
    result.success = false;
    result.err_msg = "No data found in the color image";
    return result;
  }

  // Normalize depth image
  if (depthImageRaw.type() != CV_32FC1)
  {
    if (depthImageRaw.type() == CV_16UC1)
    {
      depthImageRaw.convertTo(depthImageRaw, CV_32FC1, 0.001);
    }
    else
    {
      result.success = false;
      result.err_msg = "Depth image type must be CV_32FC1 or CV_16UC1";
      return result;
    }
  }

  const auto depthImage = depthImageRaw;

  // Create manipulable image
  auto colorImage = colorImageRaw.clone();

  // Create color mask
  cv::Mat mask;
  ColorNames colorNames;
  colorNames.createColorMask(colorImage, request.blob_color, mask);

  // Morphological operations
  float dilation_size = 1.0;
  cv::Mat morphology = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                 cv::Point(dilation_size, dilation_size));

  cv::Mat dilated, eroded;
  cv::erode(mask, eroded, morphology);
  cv::dilate(eroded, dilated, morphology);
  cv::dilate(dilated, dilated, morphology);
  cv::erode(dilated, eroded, morphology);

  // Find and sort contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(eroded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::sort(contours.begin(), contours.end(), sortContour);

  // Clear mask for output
  mask.setTo(cv::Scalar(0, 0, 0));

  uint blobNum = 0;
  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::RotatedRect rotRect = cv::minAreaRect(contours[i]);

    if (!checkValidContour(rotRect, request.min_blob_size))
    {
      continue;
    }

    bool isDesired = (request.desired_blob == blobNum);
    cv::Scalar color = isDesired ? cv::Scalar(70, 255, 70) : cv::Scalar(255, 255, 255);

    // Draw contour
    if (isDesired)
    {
      cv::drawContours(colorImage, std::vector<std::vector<cv::Point>>(1, contours[i]), -1, cv::Scalar(50, 200, 50), 4,
                       cv::LINE_8);
      cv::drawContours(mask, contours, i, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);
    }
    else
    {
      cv::drawContours(colorImage, std::vector<std::vector<cv::Point>>(1, contours[i]), -1, cv::Scalar(0, 255, 255), 1,
                       cv::LINE_8);
    }

    // Calculate centroid
    cv::Moments moment = cv::moments(contours[i]);
    if (moment.m00 == 0)
    {
      blobNum++;
      continue;
    }

    cv::Point2f momentPt(static_cast<float>(moment.m10 / moment.m00), static_cast<float>(moment.m01 / moment.m00));

    // Draw centroid and label
    cv::circle(colorImage, momentPt, 5, color, -1);
    cv::putText(colorImage, std::to_string(blobNum), cv::Point2f(momentPt.x - 10, momentPt.y - 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

    // Process desired blob
    if (isDesired)
    {
      result.success = true;
      double depth = depthImage.at<float>(momentPt);

      if (depth != 0.0)
      {
        // Compute world coordinates
        double worldX = (momentPt.x - cameraInfo.k.at(2)) * (depth / cameraInfo.k.at(0));
        double worldY = (momentPt.y - cameraInfo.k.at(5)) * (depth / cameraInfo.k.at(4));

        // Compute angle
        // double height = std::max(rotRect.size.height, rotRect.size.width);
        // double width = std::min(rotRect.size.height, rotRect.size.width);
        double angle = rotRect.angle;
        if (rotRect.size.height > rotRect.size.width)
          angle -= 90;
        if (angle < 0)
          angle += 180;

        // Compute quaternion (rotation around Z axis)
        double angleRad = (angle - 90) * CV_PI / 180.0;
        double halfAngle = angleRad / 2.0;
        double qz = std::sin(halfAngle);
        double qw = std::cos(halfAngle);

        // Fill pose
        result.centroid_pose.header = cameraInfo.header;
        result.centroid_pose.pose.position.x = worldX;
        result.centroid_pose.pose.position.y = worldY;
        result.centroid_pose.pose.position.z = depth;
        result.centroid_pose.pose.orientation.x = 0;
        result.centroid_pose.pose.orientation.y = 0;
        result.centroid_pose.pose.orientation.z = qz;
        result.centroid_pose.pose.orientation.w = qw;
      }
    }

    blobNum++;
  }

  // Convert images to messages
  cv_bridge::CvImage(cameraInfo.header, sensor_msgs::image_encodings::BGR8, colorImage).toImageMsg(result.color_img);
  cv_bridge::CvImage(cameraInfo.header, sensor_msgs::image_encodings::MONO8, colorImage).toImageMsg(result.mask);
  cv_bridge::CvImage(cameraInfo.header, sensor_msgs::image_encodings::TYPE_32FC1, colorImage)
      .toImageMsg(result.depth_img);
  result.color_img_raw = request.color_img;

  return result;
}

}  // namespace color_blob_centroid
