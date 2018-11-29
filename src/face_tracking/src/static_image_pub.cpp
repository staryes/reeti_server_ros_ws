#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>

void flagCb(const std_msgs::Int8::ConstPtr& msg )
{
    ros::Duration(0.8).sleep(); // sleep for half a second

    ROS_INFO("pause ");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    ros::Subscriber flag_sub =  nh.subscribe("/insertFlag", 1, flagCb);

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/shoushan/Desktop/reeti_white.jpg", CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);

    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        pub.publish(ros_image);
        ros::spinOnce();
        loop_rate.sleep();

    }
}
