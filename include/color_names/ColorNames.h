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

#include <stdint.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

class ColorNames
{
public:
  ColorNames(std::string encodingA = "bgr8");
  ~ColorNames();

  void colorImage(cv::Mat& imageInA, cv::Mat& imageOutA);
  void createColorMask(cv::Mat& imageA, std::string colorNameA, cv::Mat& maskA);
  static bool isValidColorName(std::string nameA);

private:
  cv::Mat m_lut;
  cv::Mat m_simpleLut;
  cv::Mat m_mask;
  cv::Mat m_kernel;
  static const char* const colorName[];
  static const cv::Vec3b colors[];

  static int32_t getColorIndex(std::string nameA);
  int32_t getColorFromIndex(uint16_t idxA);
  uint16_t computeIndex(cv::Vec3b colorA);
};
