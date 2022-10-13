#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <string>

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
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA);
    void processVelcro(geometry_msgs::msg::Pose &velcroPos);

    double m_velcroSize;
    double m_velcroSizeThreshold;
    double m_velcroAspectRatio;
    double m_velcroARThreshold;
    rclcpp::QoS m_imageQos;
    rclcpp::Service<perception_msgs::srv::VelcroDimensions>::SharedPtr service_;

    message_filters::Subscriber<sensor_msgs::msg::Image> m_depthImageSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_colorImageSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_colorInfoSub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                              sensor_msgs::msg::CameraInfo>> m_timeSyncPtr;
    cv::Mat m_mask;
    ColorNames m_colorNames;
    cv::Mat m_colorImage;
    cv::Mat m_depthImage;
    sensor_msgs::msg::CameraInfo m_imageInfo;

};