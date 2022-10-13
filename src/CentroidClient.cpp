#include "rclcpp/rclcpp.hpp"
#include "perception_msgs/srv/velcro_dimensions.hpp"

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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position: %f, %f, %f", result.get()->centroid_pose.position.x ,result.get()->centroid_pose.position.y ,result.get()->centroid_pose.position.z);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_velcro_dimensions");
  }

  rclcpp::shutdown();
  return 0;
}