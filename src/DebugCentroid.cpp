#include "velcro-centroid/DebugCentroid.h"
#define MIN_OBJECT_SIZE 25

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// save the image to a member and then process the image with what comes in the service call.
//for future streaming purposes, you would need to set up a multithreaded executor, put a wait in service, and then separate both subscribers into unique callback groups

DebugCentroid::DebugCentroid()
  : rclcpp::Node("debug_centroid")
  , m_blobSize(-1)
  , m_blobSizeThreshold(50)
  , m_blobAspectRatio(-1)
  , m_blobARThreshold(0.03)
  , m_imageQos(1)
  , m_color("black")
{
  initialize();
}

DebugCentroid::~DebugCentroid(){}

void DebugCentroid::initialize()
{
    m_imageQos.keep_last(10);
    m_imageQos.reliable();
    m_imageQos.durability_volatile();

    float dilation_size=1.0;
    m_morphology = getStructuringElement( cv::MORPH_RECT,
                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                        cv::Point( dilation_size, dilation_size ) );

    service_ = this->create_service<dex_ivr_interfaces::srv::BlobDimensions>("set_blob_dimensions", std::bind(&DebugCentroid::set_blob_dimensions, this, _1, _2));

    m_depthImageSub.subscribe(this, "/camera/aligned_depth_to_color/image_raw", m_imageQos.get_rmw_qos_profile());
    m_colorImageSub.subscribe(this, "/camera/color/image_raw", m_imageQos.get_rmw_qos_profile());
    m_colorInfoSub.subscribe(this, "/camera/color/camera_info", m_imageQos.get_rmw_qos_profile());

    m_timeSyncPtr = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                        sensor_msgs::msg::CameraInfo>>(m_colorImageSub, m_depthImageSub, m_colorInfoSub, 10);
    m_timeSyncPtr->registerCallback(std::bind(&DebugCentroid::imageCallback, this, std::placeholders::_1,
                                        std::placeholders::_2, std::placeholders::_3));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Outputting images and centroid information.\nSet aspect ratio & size to -1 to output all blobs in the specified color");
}

void DebugCentroid::set_blob_dimensions(const std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Request> request,
          std::shared_ptr<dex_ivr_interfaces::srv::BlobDimensions::Response>      response)
{
    std::string test = "Incoming Request - Aspect Ratio: " + std::to_string(request->aspect_ratio) + " Size: " + std::to_string(request->size);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), test.c_str());
    m_blobAspectRatio = request->aspect_ratio;
    m_blobARThreshold = request->aspect_ratio_threshold;
    m_blobSize = request->size;
    m_blobSizeThreshold = request->size_threshold;
    m_color = request->color;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request color -> %s", request->color.c_str());

    response->centroid_pose.position.x = -1; response->centroid_pose.position.y = -1; response->centroid_pose.position.z = -1;
}

void DebugCentroid::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& colorImMsgA,
                                         const sensor_msgs::msg::Image::ConstSharedPtr& depthImMsgA,
                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoMsgA)
{
    m_colorImage = cv::Mat(cv_bridge::toCvCopy(colorImMsgA, "bgr8")->image);    // this is the opencv encoding
    m_depthImage = cv::Mat(cv_bridge::toCvCopy(depthImMsgA)->image);    
    infoMsgA->header.frame_id; //this is just to make the compiler as this is unused but available if necessary
    if(m_depthImage.type() != CV_32FC1)
    {
        if(m_depthImage.type() == CV_16UC1)
        {
            cv::Mat(m_depthImage).convertTo(m_depthImage, CV_32FC1, 0.001);
        }
        else
        {
            printf("%s depth image type must be CV_32FC1 or CV_16UC1\n", __FUNCTION__);
        }
    }
    processBlob(); //live processing for debugging

}

void DebugCentroid::processBlob()
{
  m_mask = 0;
  m_colorNames.createColorMask(m_colorImage, m_color, m_mask);

  cv::Mat dilated, eroded;

  //erode first to delete smol noise outside obect of interest, dilate to restore object of interest to proper size
  erode(m_mask, eroded, m_morphology);
  dilate(eroded, dilated, m_morphology);

  //dilates to fill in noise inside object of interest and make it solid. erode to restore object to proper size
  dilate(dilated, dilated, m_morphology);
  erode(dilated, eroded, m_morphology);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(eroded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  for (size_t i =0; i < contours.size(); i++)
  {
    //Get rotated-rectangle boundary fit of blob from the generated image mask
    cv::RotatedRect rotRect = minAreaRect(contours[i]);
    //standardize the height and width regardless of orientation of strip or camera. height is the "longer portion of blob"
    double height = std::max(rotRect.size.height,rotRect.size.width);
    double width = std::min(rotRect.size.height,rotRect.size.width);

    // if aspect ratio is set to something specific for testing, only process those nodes
    bool processContour = false;
    if (m_blobAspectRatio == -1 &&  m_blobSize == -1 )
    {
      //eliminate noisey shadows
      if(height > MIN_OBJECT_SIZE && width > MIN_OBJECT_SIZE)
          processContour = true;
    }
    // if aspect ratio exists and is within threshold of what is expected.
    if (m_blobAspectRatio != -1 && (width / height) < (m_blobAspectRatio + m_blobARThreshold/2) &&  (width / height) > (m_blobAspectRatio - m_blobARThreshold/2))
    {
      // if size exists and meets min size requirement (close enough that it is not considered random noise)
      if(m_blobSize != -1 && height > MIN_OBJECT_SIZE && width > MIN_OBJECT_SIZE && height > (m_blobSize-m_blobSizeThreshold) && height < (m_blobSize + m_blobSizeThreshold))
      {
          processContour = true;
      }
    }


    // default behavior (Aspect/Size == -1) is to process all contours that appear
    if (processContour)
    {
      // calculate x,y coordinate of centroid
      cv::Moments moment = moments(contours[i]);
      cv::Point2f momentPt;
      if (moment.m00 != 0)
      { 
        momentPt = cv::Point2f(static_cast<float>(moment.m10 / moment.m00), static_cast<float>(moment.m01 / moment.m00));

        circle(m_colorImage, momentPt, 5, cv::Scalar(255, 255, 255), -1);
        std::string boxAR = "AR: " + std::to_string((width / height));
        std::string boxSize = "width: " + std::to_string(width) + " height: " + std::to_string(height);
        putText(m_colorImage, boxAR, cv::Point2f(momentPt.x - 25, momentPt.y - 25),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        putText(m_colorImage, boxSize, cv::Point2f(momentPt.x - 25, momentPt.y - 10),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        //Depth image stuff
        std::string  depthPrint = "depth: " + std::to_string(m_depthImage.at<float>(momentPt)) + " angle: " + std::to_string(rotRect.angle);
        putText(m_colorImage, depthPrint, cv::Point2f(momentPt.x - 25, momentPt.y + 20),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        //extreme points
        cv::Point left = *min_element(contours[i].begin(), contours[i].end(), 
            [](const cv::Point& lhs, const cv::Point& rhs) {
                return lhs.x < rhs.x;
        });
        cv::Point right = *min_element(contours[i].begin(), contours[i].end(), 
            [](const cv::Point& lhs, const cv::Point& rhs) {
                return lhs.x > rhs.x;
        });
        cv::Point up = *min_element(contours[i].begin(), contours[i].end(), 
            [](const cv::Point& lhs, const cv::Point& rhs) {
                return lhs.y < rhs.y;
        });
        cv::Point down = *min_element(contours[i].begin(), contours[i].end(), 
            [](const cv::Point& lhs, const cv::Point& rhs) {
                return lhs.y  > rhs.y;
        });


        putText(m_colorImage, "L", left,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
        putText(m_colorImage, "R", right,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
        putText(m_colorImage, "U", up,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        putText(m_colorImage, "D", down,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
      }
    }
  }

  cv::imshow("colornames", m_mask);
  cv::imshow("dilate -> eroded", eroded);
  cv::imshow("depth image", m_depthImage);
  cv::imshow("final result", m_colorImage);
  cv::waitKey(1); //set to 1 for coninuous output, set to 0 for single frame forever

}


int main(int argc, char * argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "opencv : %s\n", CV_VERSION);
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DebugCentroid>());
    rclcpp::shutdown();
    cv::destroyAllWindows();

    return 0;
}
