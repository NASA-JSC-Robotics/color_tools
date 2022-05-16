#pragma once

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <stdio.h>

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