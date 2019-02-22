#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
#include "std_msgs/UInt16MultiArray.h"


using namespace std;


static const std::string OPENCV_WINDOW = "Image window";

bool gbClickedFlag = false;
cv::Point gClickedPoint;


void mouseEvent(int evt, int x, int y, int flags, void* param)
{
    //cv::Mat* rgb = (cv::Mat*) param;
  if (evt == CV_EVENT_LBUTTONDOWN)
    {
        //cv::Mat hsv;
      //cv::cvtColor(*rgb, hsv, cv::COLOR_BGR2HSV);
      std::printf("%d %d:\n",
                  x, y
                  );

      gbClickedFlag = true;
      gClickedPoint.x = x;
      gClickedPoint.y = y;
    }
}


// ROS -> OpenCV -> RGB-Depth?!
class ImageConverter{
  ros::NodeHandle nh_;

  // start/stop

  ros::Subscriber stop_sub_;
  // Camera
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;

  // output

  tf::TransformBroadcaster br_; // XXX question
  ros::Publisher goal_pub_;

  // dynamic reconfigure
  //  ros::Subscriber reconfig_sub_;
  //ros::Subscriber threshold_sub_;

  int counter;

  uint h_min;
  uint h_max;
  uint s_min;
  uint s_max;
  uint v_min;
  uint v_max;

public:
  ImageConverter()
    : it_(nh_) {
    // subscribe to updated color threshold
    //threshold_sub_ = nh_.subscribe<blob_detect::ColorThreshold>("/color_threshold", 1, &ImageConverter::thresholdCB, this);

    //Publishes position of center of pass

    //goal_pub_ = nh_.advertise<geometry_msgs::Twist>("goal_pose",1);
    goal_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("clicked_point",1);


    h_min = 0;
    h_max = 10;
    s_min = 0;
    s_max = 255;
    v_min = 0;
    v_max = 200;

      init();
  }

  void init(){


    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/blob_detect/output_video",1);

    //    reconfig_sub_ = nh_.subscribe<warmup::LidarCone>("dynamic_reconfigure/sensor_cone", 1, &ImageConverter::reconfigCb, this);
    //cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::processMouseEvent);
    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);


    /* Calibrated values for bravobot based on dynamic reconfigure test */
    //    rightEdgeScanIndex_ = 358;
    //  leftEdgeScanIndex_ = 187;
  }

  void sleep(){
    image_sub_.shutdown();
    stop_sub_.shutdown();
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
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
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(10);

    cv::setMouseCallback(OPENCV_WINDOW, mouseEvent, &cv_ptr->image); // test


    if ( gbClickedFlag == true )
      {

        std_msgs::UInt16MultiArray msg_output;
        msg_output.data.resize(2);

        msg_output.data[0] = gClickedPoint.x;
        msg_output.data[1] = gClickedPoint.y;

        goal_pub_.publish(msg_output);

        gbClickedFlag = false;
      }


    ros::NodeHandle nh("wo");
    ros::Publisher pub = nh.advertise<std_msgs::String>("wowo_hello", 5);
    std_msgs::String str;
    str.data = "hello world";
    pub.publish(str);

  }

  //Find center of mass of legs by taking the average of the white points in the image.
  cv::Point center_of_mass (cv::Mat input){
    float xsum = 0;
    float ysum = 0;
    float numPoints = 0;
    float width = input.cols;
    float height = input.rows;

    for (int j = 0; j < height; j++){
      for (int i = 0; i < width; i++){
        cv::Scalar color = input.at<uchar>(cv::Point(i, j));
        if (color.val[0] > 200){
          xsum += i;
          ysum += j;
          numPoints ++;
        }
      }
    }

    cv::Point com;
    com.x = (int)((xsum/numPoints) - (width/2));
    com.y = (int)((-1*ysum/numPoints) + (height/2));

    return com;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
