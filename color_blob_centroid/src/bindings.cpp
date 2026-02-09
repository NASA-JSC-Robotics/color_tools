#include <nanobind/nanobind.h>
#include "color_blob_centroid/color_blob_centroid.hpp"
#include "color_blob_centroid/bindings.hpp"

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(bindings, m) {
  m.doc() = "Nanobind python bindings for colorblob centroid.";

  m.def(
      "process_blobs",
      [](nb::handle color_image_msg,
         nb::handle depth_image_msg,
         nb::handle camera_info_msg,
         nb::handle blob_request_msg,
         nb::object blob_result_class) {
        auto color_image = pyToCppMsg<sensor_msgs::msg::Image>(color_image_msg);
        auto depth_image = pyToCppMsg<sensor_msgs::msg::Image>(depth_image_msg);
        auto camera_info = pyToCppMsg<sensor_msgs::msg::CameraInfo>(camera_info_msg);
        auto request = pyToCppMsg<color_tools_msgs::msg::BlobRequest>(blob_request_msg);
        auto result = color_blob_centroid::processBlobs(color_image, depth_image, camera_info, request);
        return cppToPyMsg(result, blob_result_class);
      },
      "color_image"_a,
      "depth_image"_a,
      "camera_info"_a,
      "request"_a,
      "result_class"_a,
      "Process color and depth images to find colored blobs.");
}
