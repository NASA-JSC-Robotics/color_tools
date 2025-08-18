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

#include "color_tools_msgs/srv/blob_dimensions.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: blob_dimensions_client <aspect ratio> <aspect "
                                              "threshold> <size> <size threshold>");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("blob_dimensions_client");
  rclcpp::Client<color_tools_msgs::srv::BlobDimensions>::SharedPtr client =
      node->create_client<color_tools_msgs::srv::BlobDimensions>("set_blob_dimensions");

  auto request = std::make_shared<color_tools_msgs::srv::BlobDimensions::Request>();
  request->aspect_ratio = atof(argv[1]);
  request->aspect_ratio_threshold = atof(argv[2]);
  request->size = atof(argv[3]);
  request->size_threshold = atof(argv[4]);
  request->color = (argv[5]);

  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    static tf2_ros::TransformBroadcaster tf_bc = tf2_ros::TransformBroadcaster(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position: %f, %f, %f, orientation: %f, %f, %f, %f",
                result.get()->centroid_pose.pose.position.x, result.get()->centroid_pose.pose.position.y,
                result.get()->centroid_pose.pose.position.z, result.get()->centroid_pose.pose.orientation.x,
                result.get()->centroid_pose.pose.orientation.y, result.get()->centroid_pose.pose.orientation.z,
                result.get()->centroid_pose.pose.orientation.w);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "color_blob_centroid";
    t.transform.translation.x = result.get()->centroid_pose.pose.position.x;
    t.transform.translation.y = result.get()->centroid_pose.pose.position.y;
    t.transform.translation.z = result.get()->centroid_pose.pose.position.z;
    t.transform.rotation.x = result.get()->centroid_pose.pose.orientation.x;
    t.transform.rotation.y = result.get()->centroid_pose.pose.orientation.y;
    t.transform.rotation.z = result.get()->centroid_pose.pose.orientation.z;
    t.transform.rotation.w = result.get()->centroid_pose.pose.orientation.w;
    tf_bc.sendTransform(t);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_blob_dimensions");
  }

  rclcpp::shutdown();
  return 0;
}
