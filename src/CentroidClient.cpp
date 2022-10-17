#include "rclcpp/rclcpp.hpp"
#include "perception_msgs/srv/velcro_dimensions.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: velcro_dimensions_client <aspect ratio> <size>");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("velcro_dimensions_client");
  rclcpp::Client<perception_msgs::srv::VelcroDimensions>::SharedPtr client =
    node->create_client<perception_msgs::srv::VelcroDimensions>("set_velcro_dimensions");


  auto request = std::make_shared<perception_msgs::srv::VelcroDimensions::Request>();
  request->aspect_ratio = atof(argv[1]);
  request->size = atof(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    //rclcpp::QoS qos = rclcpp::QoS(1);
    static tf2_ros::TransformBroadcaster tf_bc = tf2_ros::TransformBroadcaster(node);
    //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position: %f, %f, %f", result.get()->centroid_pose.position.x ,result.get()->centroid_pose.position.y ,result.get()->centroid_pose.position.z);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "happyframe";
    t.transform.translation.x = result.get()->centroid_pose.position.x;
    t.transform.translation.y = result.get()->centroid_pose.position.y;
    t.transform.translation.z = result.get()->centroid_pose.position.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, 30);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_bc.sendTransform(t);

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_velcro_dimensions");
  }

  rclcpp::shutdown();
  return 0;
}