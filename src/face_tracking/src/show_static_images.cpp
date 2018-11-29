#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
//#include "blob_detect/ColorThreshold.h"
//#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Int8.h>


using namespace std;

//#include "slic.h"


static const std::string OPENCV_WINDOW_LEFT = "Image window left";
static const std::string OPENCV_WINDOW_RIGHT = "Image window right";

bool gbClickedFlag = false;
cv::Point gClickedPoint;

void mouseEvent(int evt, int x, int y, int flags, void* param)
{
    //cv::Mat* rgb = (cv::Mat*) param;
  if (evt == CV_EVENT_LBUTTONDOWN)
    {
        //cv::Mat hsv;
      //cv::cvtColor(*rgb, hsv, cv::COLOR_BGR2HSV);
      std::printf("%d %d:\n",// %d %d %d; %d %d %d\n",
                  x, y//,
                  //(int)(*rgb).at<cv::Vec3b>(y, x)[0],
                  //(int)(*rgb).at<cv::Vec3b>(y, x)[1],
                  //(int)(*rgb).at<cv::Vec3b>(y, x)[2]//,
                  //(int)(hsv).at<cv::Vec3b>(y, x)[0],
                  //(int)(hsv).at<cv::Vec3b>(y, x)[1],
                  //(int)(hsv).at<cv::Vec3b>(y, x)[2]
                  );
      ROS_INFO("Left button!");
      //gbClickedFlag = true;
      //gClickedPoint.x = x;
      //gClickedPoint.y = y;
    }
  else if (evt == CV_EVENT_RBUTTONDOWN)
  {
      ROS_INFO("Right button!");
  }
}


// ROS -> OpenCV -> RGB-Depth?!
class ImageConverter{
  ros::NodeHandle nh_;

  // start/stop
  //ros::Subscriber start_sub_;
  //ros::Subscriber stop_sub_;
  // Camera
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

    //ros::Subscriber flag_sub_;


public:
  ImageConverter()
    : it_(nh_) {
    // subscribe to updated color threshold
    //threshold_sub_ = nh_.subscribe<blob_detect::ColorThreshold>("/color_threshold", 1, &ImageConverter::thresholdCB, this);

    //start_sub_ = nh_.subscribe<std_msgs::Bool>("start", 1, &ImageConverter::startCb, this);

    //Publishes position of center of pass
    //com_pub_ = nh_.advertise<geometry_msgs::Point>("center_of_mass", 1);

    //goal_pub_ = nh_.advertise<geometry_msgs::Twist>("goal_pose",1);
    //goal_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("goal_pose",1);


    init();

  }

  void init(){
      //stop_sub_ = nh_.subscribe<std_msgs::Bool>("stop", 1, &ImageConverter::stopCb, this);

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/static_image", 1, &ImageConverter::imageCb, this);
    //flag_sub_ = nh_.subscribe("/insertFlag", 1, &ImageConverter::flagCb, this);
    //image_pub_ = it_.advertise("/blob_detect/output_video",1);

    // Subscribe to laser scan data
    //    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &ImageConverter::laserScanCallback, this);

    //    reconfig_sub_ = nh_.subscribe<warmup::LidarCone>("dynamic_reconfigure/sensor_cone", 1, &ImageConverter::reconfigCb, this);
    //cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::processMouseEvent);
    cv::namedWindow(OPENCV_WINDOW_LEFT, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(OPENCV_WINDOW_RIGHT, CV_WINDOW_AUTOSIZE);

    /* Calibrated values for bravobot based on dynamic reconfigure test */
    //    rightEdgeScanIndex_ = 358;
    //  leftEdgeScanIndex_ = 187;
  }

  void sleep(){
    image_sub_.shutdown();
    //stop_sub_.shutdown();
  }

  ~ImageConverter()
  {
      cv::destroyWindow(OPENCV_WINDOW_LEFT);
      cv::destroyWindow(OPENCV_WINDOW_RIGHT);
  }

    // void flagCb(const std_msgs::Int8::ConstPtr& msg )
    // {
    //     cv::imshow(OPENCV_WINDOW_LEFT, image0);
    //     cv::imshow(OPENCV_WINDOW_RIGHT, image0);
    //     cv::waitKey(30);

    //     if (msg->data == 1)
    //     {
    //         cv::imshow(OPENCV_WINDOW_LEFT, image1);
    //     }
    //     else if (msg->data == 2)
    //     {
    //         cv::imshow(OPENCV_WINDOW_RIGHT, image2);
    //     }
    //     cv::waitKey(200);

    //     cv::imshow(OPENCV_WINDOW_LEFT, image0);
    //     cv::imshow(OPENCV_WINDOW_RIGHT, image0);

    //     char k;
    //     k = cv::waitKey(0);

    //     if(k == 'v')
    //         ROS_INFO("key 'v' pressed");
    //     else if(k == 'u')
    //         ROS_INFO("key 'u' pressed");


    // }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_LEFT, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW_RIGHT, cv_ptr->image);
    cv::waitKey(30);


    cv::setMouseCallback(OPENCV_WINDOW_LEFT, mouseEvent, &cv_ptr->image); // test



    //
    //ros::NodeHandle nh("wo");
    //ros::Publisher pub = nh.advertise<std_msgs::String>("wowo_hello", 5);
    //std_msgs::String str;
    //str.data = "hello world";
    //pub.publish(str);

  }


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
