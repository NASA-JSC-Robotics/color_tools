#include "color_names/ColorNames.h"
#include "color_names/ColorNameLut.h"

const char *const ColorNames::colorName[] = {
    "black", "blue",   "brown", "grey",  "green", "orange",
    "pink",  "purple", "red",   "white", "yellow"};

const cv::Vec3b ColorNames::colors[] = {
    cv::Vec3b(0, 0, 0),       cv::Vec3b(255, 0, 0),   cv::Vec3b(0, 75, 150),
    cv::Vec3b(127, 127, 127), cv::Vec3b(0, 255, 0),   cv::Vec3b(0, 165, 255),
    cv::Vec3b(80, 75, 100),   cv::Vec3b(255, 0, 127), cv::Vec3b(0, 0, 255),
    cv::Vec3b(255, 255, 255), cv::Vec3b(0, 255, 255)};

ColorNames::ColorNames(std::string encodingA) {
  m_kernel = cv::getStructuringElement(0, cv::Size(2 * 2 + 1, 2 * 2 + 1),
                                       cv::Point(2, 2));
  if (encodingA == "rgb8") {
    m_lut = cv::Mat(32768, 14, CV_32F, RgbLutData);
  } else {
    m_lut = cv::Mat(32768, 14, CV_32F, BgrLutData);
  }
  m_simpleLut = cv::Mat(32768, 1, CV_8UC1);
  for (int32_t r = 0; r < m_lut.rows; ++r) {
    m_simpleLut.at<uint8_t>(r) = static_cast<uint8_t>(getColorFromIndex(r));
  }
}

ColorNames::~ColorNames() {}

bool ColorNames::isValidColorName(std::string nameA) {
  bool valid = true;
  int32_t idx = getColorIndex(nameA);
  if (idx < 0) {
    valid = false;
  }
  return (valid);
}

int32_t ColorNames::getColorIndex(std::string nameA) {
  int32_t idx = -1;
  std::string name = nameA;
  for (size_t i = 0; i < nameA.length(); i++) {
    name[i] = std::tolower(nameA[i]);
  }
  for (size_t i = 0; i < 11; ++i) {
    if (ColorNames::colorName[i] == name) {
      idx = i;
      break;
    }
  }
  return (idx);
}

uint16_t ColorNames::computeIndex(cv::Vec3b colorA) {
  uint16_t idx = static_cast<uint16_t>(
      std::floor(static_cast<float>(colorA[2]) / 8.0) +
      (32.0 * std::floor(static_cast<float>(colorA[1]) / 8.0)) +
      (32.0 * 32 * std::floor(static_cast<float>(colorA[0]) / 8.0)));

  return (idx);
}

int32_t ColorNames::getColorFromIndex(uint16_t idxA) {
  double min, max;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(m_lut(cv::Rect(3, idxA, 11, 1)), &min, &max, &minLoc, &maxLoc);

  return (maxLoc.x);
}

void ColorNames::createColorMask(cv::Mat &imageA, std::string colorNameA,
                                 cv::Mat &maskA) {
  static cv::Mat image;
  maskA.create(imageA.size(), CV_8UC1);
  maskA = 0;

  cv::cvtColor(imageA, image, cv::COLOR_BGR2BGR555);

  int32_t maskColorIdx = getColorIndex(colorNameA);

  if (maskColorIdx < 0) {
    fprintf(stderr, "Invalid color name %s\n", colorNameA.c_str());
  } else {
    for (int32_t r = 0; r < image.rows; ++r) {
      for (int32_t c = 0; c < image.cols; ++c) {
        int32_t colorIdx = m_simpleLut.at<uint8_t>(image.at<uint16_t>(r, c));
        if (colorIdx == maskColorIdx) {
          maskA.at<uint8_t>(r, c) = 255;
        }
      }
    }
    cv::morphologyEx(maskA, maskA, cv::MORPH_OPEN, m_kernel);
  }
}

void ColorNames::colorImage(cv::Mat &imageInA, cv::Mat &imageOutA) {
  static cv::Mat image;
  cv::cvtColor(imageInA, image, cv::COLOR_BGR2BGR555);

  imageOutA.create(imageInA.size(), CV_8UC3);
  for (int32_t r = 0; r < imageInA.rows; ++r) {
    for (int32_t c = 0; c < imageInA.cols; ++c) {
      int32_t colorIdx = m_simpleLut.at<uint8_t>(image.at<uint16_t>(r, c));
      if (colorIdx < 11) {
        imageOutA.at<cv::Vec3b>(r, c) = colors[colorIdx];
      } else {
        printf("idx = %u\n", colorIdx);
      }
    }
  }
}
