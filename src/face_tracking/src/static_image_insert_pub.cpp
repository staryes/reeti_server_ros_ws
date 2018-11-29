#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>

class ImagePublisher
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber flag_sub_;

    //ros::Rate loop_rate(10);

    cv::String image1_name = "/home/staryes/Desktop/CIMG0033.JPG";
    cv::String image2_name = "/home/staryes/Desktop/CIMG0178.JPG";
    cv::String image0_name = "/home/staryes/Desktop/OperaMeganeko.png";

    sensor_msgs::ImagePtr imsg1;
    sensor_msgs::ImagePtr imsg2;
    sensor_msgs::ImagePtr imsg0;
//    sensor_msgs::ImagePtr

    void loadImages(void)
    {
        cv::Mat image1 = cv::imread(this->image1_name);
        imsg1= cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();

        cv::Mat image2 = cv::imread(this->image2_name);
        imsg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

        cv::Mat image0 = cv::imread(this->image0_name);
        imsg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image0).toImageMsg();
    }


public:
    ImagePublisher()
        : it_(nh_)
    {
        image_pub_ = it_.advertise("/static_image", 1);
        flag_sub_ = nh_.subscribe("/insertFlag", 1, &ImagePublisher::flagCb, this);

        loadImages();
    }

    ~ImagePublisher()
    {
    }

    void flagCb(const std_msgs::Int8::ConstPtr& msg )
    {
        ros::Duration(0.5).sleep(); // sleep for half a second
        if (msg->data == 1)
        {
            image_pub_.publish(imsg1);

            ros::Duration(0.2).sleep();
        }
        else if (msg->data == 2)
        {
            image_pub_.publish(imsg2);
            ros::Duration(0.2).sleep();
        }

        image_pub_.publish(imsg0);
    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_insert_publisher");

    ImagePublisher ip;
    ros::spin();

    return 0;
}
