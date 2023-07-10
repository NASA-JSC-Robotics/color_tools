#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <string>

#include <color_names/ColorNames.h>
#include "dex_ivr_interfaces/srv/blob_dimensions.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>

class DebugCentroid : public rclcpp::Node
{
public:
    DebugCentroid();
    ~DebugCentroid();

private:
    void initialize();
    void set_blob_dimensions(const std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Request> request,
      std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Response>      response);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA);
    void processBlob();

    double m_blobSize;
    double m_blobSizeThreshold;
    double m_blobAspectRatio;
    double m_blobARThreshold;
    std::string m_prefix;
    rclcpp::QoS m_imageQos;
    rclcpp::Service<dex_ivr_interfaces::srv::BlobDimensions>::SharedPtr service_;

    message_filters::Subscriber<sensor_msgs::msg::Image> m_depthImageSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_colorImageSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_colorInfoSub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                              sensor_msgs::msg::CameraInfo>> m_timeSyncPtr;
    cv::Mat m_mask;
    ColorNames m_colorNames;
    cv::Mat m_colorImage;
    cv::Mat m_depthImage;
    cv::Mat m_morphology;
    std::string m_color;
    sensor_msgs::msg::CameraInfo m_imageInfo;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcasterPtr = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};