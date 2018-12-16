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
    ros::init(argc, argv, "blank_image_publisher");
    ros::NodeHandle nh_;

    ros::Subscriber flag_sub =  nh_.subscribe("/insertFlag", 1, flagCb);

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/shoushan/Desktop/reeti_white.jpg", CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub0 = nh_.advertise<sensor_msgs::Image>("/static_image0", 1);
    ros::Publisher pub1 = nh_.advertise<sensor_msgs::Image>("/static_image1", 1);

    ros::Rate loop_rate(1);

    while(nh_.ok())
    {
        pub0.publish(ros_image);
        pub1.publish(ros_image);
        ros::spinOnce();
        loop_rate.sleep();

    }
}
