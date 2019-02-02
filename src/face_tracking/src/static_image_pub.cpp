#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>

cv::String image1_name = "/home/staryes/reeti_server_ros_ws/reeti_u.jpg";
cv::String image2_name = "/home/staryes/reeti_server_ros_ws/reeti_v.jpg";
cv::String image0_name = "/home/staryes/reeti_server_ros_ws/reeti_white.jpg";

int flag = -1;

void flagCb(const std_msgs::Int8::ConstPtr& msg )
{
    ros::Duration(1).sleep(); // sleep for 0.5 second

    flag = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_image_publisher");
    ros::NodeHandle nh_;

    ros::Subscriber flag_sub =  nh_.subscribe("/insertFlag", 1, flagCb);
    
    sensor_msgs::ImagePtr imsg1;
    sensor_msgs::ImagePtr imsg2;
    sensor_msgs::ImagePtr imsg0;

    cv::Mat image1 = cv::imread(image1_name);
    imsg1= cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();

    cv::Mat image2 = cv::imread(image2_name);
    imsg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

    cv::Mat image0 = cv::imread(image0_name);
    imsg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image0).toImageMsg();

    ros::Publisher pub0 = nh_.advertise<sensor_msgs::Image>("/static_image0", 1);
    ros::Publisher pub1 = nh_.advertise<sensor_msgs::Image>("/static_image1", 1);

    ros::Rate loop_rate(20 ); // 50 ms

    while(nh_.ok())
    {
        if (flag == 1)
        {
            flag = -1;
            pub0.publish(imsg1);
            pub1.publish(imsg0);
            //ROS_DEBUG("0 u");
        }
        else if (flag == 2)
        {
            flag = -1;
            pub0.publish(imsg2);
            pub1.publish(imsg0);
            //ROS_INFO("0 v");
        }
        else if(flag == 3)
        {
            flag = -1;
            pub0.publish(imsg0);
            pub1.publish(imsg1);
        }
        else if (flag == 4)
        {
            flag = -1;
            pub0.publish(imsg0);
            pub1.publish(imsg2);
        }
        else
        {
            pub0.publish(imsg0);
            pub1.publish(imsg0);
            flag = -1;
        }
        
        ros::spinOnce();
        loop_rate.sleep();

    }
}
