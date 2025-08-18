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

#include <opencv2/opencv.hpp>

#include "color_names/ColorNames.h"

int32_t main(int32_t argc, char** argv)
{
  char key = 0x00;
  cv::VideoCapture camera(0);
  ColorNames colorNames("bgr8");
  std::string color = "yellow";
  if (!camera.isOpened())
  {
    std::cerr << "ERROR: Could not open camera" << std::endl;
    return 1;
  }
  if (argc > 1)
  {
    color = argv[1];
  }

  cv::Mat image;
  cv::Mat mask;

  while (key != 'q')
  {
    // show the image on the window
    camera.read(image);
    colorNames.createColorMask(image, color, mask);
    cv::Mat display;
    cv::cvtColor(image, display, cv::COLOR_BGR2GRAY);
    cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    image.copyTo(display, mask);
    cv::imshow("grey", display);

    key = cv::waitKey(1);
  }
  return 0;
}
