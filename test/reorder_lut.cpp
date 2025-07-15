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

#include <stdint.h>
#include <opencv2/opencv.hpp>

#include "color_names/ColorNameLut.h"

int32_t main(int32_t, char**)
{
  cv::Mat rgbLut = cv::Mat(32768, 14, CV_32F, RgbLutData);
  cv::Mat bgrLut = cv::Mat::zeros(32768, 14, CV_32F);
  for (int32_t row = 0; row < rgbLut.rows; ++row)
  {
    uint16_t red = (0x7C00 & (uint16_t)row) >> 10;
    uint16_t grn = (0x03E0 & (uint16_t)row) >> 5;
    uint16_t blu = (0x001F & (uint16_t)row);
    uint32_t bgr = red + grn * 32 + blu * 32 * 32;
    rgbLut.row(row).copyTo(bgrLut.row(bgr));
  }
  cv::FileStorage fs("bgrlutdata.json", cv::FileStorage::WRITE);
  fs << "rgb_lut" << bgrLut;
  fs.release();
}
