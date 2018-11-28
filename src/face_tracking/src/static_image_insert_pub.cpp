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

    cv::Mat image1;
    cv::Mat image2;
    cv::Mat image0;

//    sensor_msgs::ImagePtr imsg1;
//    sensor_msgs::ImagePtr

    void loadImages(void)
    {
        this->image1 = cv::imread(this->image1_name);
        this->image2 = cv::imread(this->image2_name);
        this->image0 = cv::imread(this->image0_name);
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
        if (msg->data == 1)
        {
            sensor_msgs::ImagePtr imsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();
            image_pub_.publish(imsg);
        }
        else if (msg->data == 2)
        {
            sensor_msgs::ImagePtr imsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();
            image_pub_.publish(imsg);
        }

        else
        {
            sensor_msgs::ImagePtr imsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image0).toImageMsg();
            image_pub_.publish(imsg);
        }


    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_insert_publisher");

    ImagePublisher ip;
    ros::spin();

    return 0;
}

/*
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/staryes/Desktop/OperaMeganeko.png", CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    cv_bridge::CvImage ano_image;
    ano_image.image = cv::imread("/home/staryes/Desktop/CIMG0178.JPG", CV_LOAD_IMAGE_COLOR);
    ano_image.encoding = "bgr8";
    sensor_msgs::Image ano_r_image;
    ano_image.toImageMsg(ano_r_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);
    ros::Rate loop_rate(5);

    int count = 0;
    while(nh.ok())
    {
        if (count%20 == 0)
            pub.publish(ano_r_image);
        else
            pub.publish(ros_image);
        loop_rate.sleep();

        count++;
        //ros::spin();
    }
}
*/
