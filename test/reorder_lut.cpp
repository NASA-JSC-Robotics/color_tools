
#include <opencv2/opencv.hpp>
#include <stdint.h>

#include "color_names/ColorNameLut.h"

int32_t main(int32_t, char**)
{
    cv::Mat rgbLut = cv::Mat(32768, 14, CV_32F, RgbLutData);
    cv::Mat bgrLut = cv::Mat::zeros(32768, 14, CV_32F);
    for(int32_t row = 0; row < rgbLut.rows; ++row)
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