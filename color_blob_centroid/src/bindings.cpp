#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "color_blob_centroid/bindings.hpp"
#include "color_blob_centroid/color_blob_centroid.hpp"

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(bindings, m)
{
  m.doc() = "Nanobind python bindings for colorblob centroid.";

  // Configure types, we have to do
  auto sensor_msgs = nb::module_::import_("sensor_msgs.msg");
  auto geometry_msgs = nb::module_::import_("geometry_msgs.msg");
  nb::object Image = sensor_msgs.attr("Image");
  nb::object CameraInfo = sensor_msgs.attr("CameraInfo");
  nb::object PoseStamped = geometry_msgs.attr("PoseStamped");

  nb::class_<color_blob_centroid::BlobRequest>(m, "BlobRequest")
      .def(nb::init<>())
      .def_rw("blob_color", &color_blob_centroid::BlobRequest::blob_color)
      .def_rw("min_blob_size", &color_blob_centroid::BlobRequest::min_blob_size)
      .def_rw("desired_blob", &color_blob_centroid::BlobRequest::desired_blob)
      // We can't access these directly as ROS types so they must be converted on the fly
      .def("set_color_img", [Image](color_blob_centroid::BlobRequest& self,
                                    nb::handle py_msg) { self.color_img = pyToCppMsg<sensor_msgs::msg::Image>(py_msg); })
      .def("set_depth_img", [Image](color_blob_centroid::BlobRequest& self,
                                    nb::handle py_msg) { self.depth_img = pyToCppMsg<sensor_msgs::msg::Image>(py_msg); })
      .def("set_camera_info", [CameraInfo](color_blob_centroid::BlobRequest& self, nb::handle py_msg) {
        self.camera_info = pyToCppMsg<sensor_msgs::msg::CameraInfo>(py_msg);
      });

  nb::class_<color_blob_centroid::BlobResult>(m, "BlobResult")
      .def(nb::init<>())
      .def_ro("success", &color_blob_centroid::BlobResult::success)
      .def_ro("err_msg", &color_blob_centroid::BlobResult::err_msg)
      // We can't access these directly as ROS types so they must be converted on the fly
      .def("get_centroid_pose",
           [PoseStamped](const color_blob_centroid::BlobResult& self) {
             return cppToPyMsg(self.centroid_pose, PoseStamped);
           })
      .def("get_color_img",
           [Image](const color_blob_centroid::BlobResult& self) { return cppToPyMsg(self.color_img, Image); })
      .def("get_mask", [Image](const color_blob_centroid::BlobResult& self) { return cppToPyMsg(self.mask, Image); })
      .def("get_color_img_raw",
           [Image](const color_blob_centroid::BlobResult& self) { return cppToPyMsg(self.color_img_raw, Image); })
      .def("get_depth_img",
           [Image](const color_blob_centroid::BlobResult& self) { return cppToPyMsg(self.depth_img, Image); });

  m.def("process_blobs", &color_blob_centroid::processBlobs, "request"_a,
        "Process color and depth images to find colored blobs.");
}
