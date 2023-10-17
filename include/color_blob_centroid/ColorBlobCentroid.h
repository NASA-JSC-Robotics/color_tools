#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/set_bool.hpp"
#include <string>

#include <color_names/ColorNames.h>
#include "dex_ivr_interfaces/srv/blob_dimensions.hpp"
#include "dex_ivr_interfaces/srv/blob_centroid.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>

class ColorBlobCentroid : public rclcpp::Node
{
public:
    ColorBlobCentroid();
    ~ColorBlobCentroid();
    static bool sortContour(std::vector<cv::Point> a, std::vector<cv::Point> b)
    {
      cv::Rect rectA = cv::boundingRect(a);
      cv::Rect rectB = cv::boundingRect(b);

      if (abs(rectA.y - rectB.y) <= 25)
          return (rectA.x < rectB.x);

      return (rectA.y < rectB.y);
    }

private:
    void initialize();
    /* Helper */
    bool sendMockHardwareTransform(geometry_msgs::msg::PoseStamped &blobPos); //makes fake transform position 0.5 units Z direction from camera optical frame
    void processContour(geometry_msgs::msg::PoseStamped &blobPos, sensor_msgs::msg::Image &blobImg, cv::Point2f momentPt, cv::RotatedRect rotRect); //calculates final realworld coordinates of specific contour, writes data to image
    bool checkValidContour(cv::RotatedRect rotRect); //verify that a contour is within thresholds set by services
    void outputContour(geometry_msgs::msg::PoseStamped &blobPos, sensor_msgs::msg::Image &blobImg, double worldX, double worldY, double depth, double angle); //using computed blob metrics, output for service using first two parameters, and publish transform of blob location
    /* Services */
    void color_blob_find(const std::shared_ptr<dex_ivr_interfaces::srv::BlobCentroid::Request> request,
      std::shared_ptr<dex_ivr_interfaces::srv::BlobCentroid::Response>      response);
    void color_set_blob_dimensions(const std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Request> request,
      std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Response>      response);
    void toggle_continuous(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    /* Core Processing */
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA);
    void processBlobs(geometry_msgs::msg::PoseStamped &blobPos,sensor_msgs::msg::Image &blobImg); //iterates through all color blobs in image and filters them with openCV & the thresholds specified by service

    //Blob filtering parameters to maintain between service calls
    double m_minBlobSize;
    double m_blobSize;
    double m_blobSizeThreshold;
    double m_blobAspectRatio;
    double m_blobARThreshold;
    std::string m_prefix;
    //ROS stuff
    rclcpp::QoS m_imageQos;
    rclcpp::Service<dex_ivr_interfaces::srv::BlobDimensions>::SharedPtr m_color_srv; //configures the parameters of the color blob detection: aspect ratio, size, color
    rclcpp::Service<dex_ivr_interfaces::srv::BlobCentroid>::SharedPtr m_color_simple_srv; //configures the parameters of the color blob detection: min blob size and color
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_processing_srv; //turn on/off continuous image processing

    message_filters::Subscriber<sensor_msgs::msg::Image> m_depthImageSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_colorImageSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_colorInfoSub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                              sensor_msgs::msg::CameraInfo>> m_timeSyncPtr;
    //Image checkpoints                                          
    cv::Mat m_mask;
    ColorNames m_colorNames;
    cv::Mat m_colorImage;
    cv::Mat m_depthImage;
    cv::Mat m_morphology;
    //Service call thresholds
    std::string m_color;
    bool m_continuousColor; //flag for continuous processing of color
    bool m_mockHardware;
    bool m_showImage;
    bool m_debugMode;
    uint m_desiredBlob;
    uint m_blobNum;
    sensor_msgs::msg::CameraInfo m_imageInfo;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcasterPtr = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
};