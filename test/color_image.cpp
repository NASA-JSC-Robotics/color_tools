#include <opencv2/opencv.hpp>

#include "color_names/ColorNames.h"

int32_t main(int32_t, char **) {
  char key = 0x00;
  cv::VideoCapture camera(0);
  ColorNames colorNames;
  if (!camera.isOpened()) {
    std::cerr << "ERROR: Could not open camera" << std::endl;
    return 1;
  }
  cv::namedWindow("image", cv::WINDOW_NORMAL);

  cv::Mat image;
  cv::Mat colorImage;

  while (key != 'q') {
    // show the image on the window
    camera.read(image);
    std::chrono::time_point<std::chrono::steady_clock> start =
        std::chrono::steady_clock::now();
    colorNames.colorImage(image, colorImage);
    std::chrono::time_point<std::chrono::steady_clock> end =
        std::chrono::steady_clock::now();
    std::cout << "Elapsed time in microseconds: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << " ms" << std::endl;
    cv::imshow("Image", colorImage);

    key = cv::waitKey(1);
  }
  return 0;
}
