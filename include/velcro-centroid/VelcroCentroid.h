#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <color_names/ColorNames.h>
#include "perception_msgs/srv/velcro_dimensions.hpp"
#include "std_msgs/msg/string.hpp"



class VelcroCentroid : public rclcpp::Node
{
    public:
        VelcroCentroid();
        ~VelcroCentroid();

    private:
        void initialize();
        void set_velcro_dimensions(const std::shared_ptr<perception_msgs::srv::VelcroDimensions::Request> request,
          std::shared_ptr<perception_msgs::srv::VelcroDimensions::Response>      response);
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & colorImMsgA);
        void processVelcro();

        rclcpp::QoS m_imageQos;
        rclcpp::Service<perception_msgs::srv::VelcroDimensions>::SharedPtr service_;
        image_transport::Subscriber m_imageSub;
        cv::Mat m_colorImage;

};