#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "ros/console.h"

#include "reetiros/MoveEars.h"
#include "reetiros/Say.h"
#include "reetiros/MoveNeck.h"

#include "reetiros/reetiEyesPose.h"
#include "reetiros/reetiNeckPose.h"
#include "reetiros/reetiPose.h"

#define KEYCODE_RIGHT_ARROW 0x43 
#define KEYCODE_LEFT_ARROW 0x44
#define KEYCODE_UP_ARROW 0x41
#define KEYCODE_DOWN_ARROW 0x42

#define Keycode_1 0x31
#define Keycode_2 0x32
#define Keycode_3 0x33
#define Keycode_4 0x34
#define Keycode_5 0x35
#define Keycode_6 0x36
#define Keycode_7 0x37
#define Keycode_8 0x38
#define Keycode_9 0x39
#define Keycode_0 0x30

#define KEYCODE_q 0x71
#define KEYCODE_w 0x77
#define KEYCODE_e 0x65
#define KEYCODE_r 0x72
#define KEYCODE_t 0x74
#define KEYCODE_y 0x79
#define KEYCODE_u 0x75
#define KEYCODE_i 0x69
#define KEYCODE_o 0x6F
#define KEYCODE_p 0x70

#define KEYCODE_a 0x61
#define KEYCODE_s 0x73
#define KEYCODE_d 0x64
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_j 0x6A
#define KEYCODE_k 0x6B
#define KEYCODE_l 0x6C

#define KEYCODE_z 0x7A
#define KEYCODE_x 0x78
#define KEYCODE_c 0x63
#define KEYCODE_v 0x76
#define KEYCODE_b 0x62
#define KEYCODE_n 0x6E
#define KEYCODE_m 0x6D

class ReetiROSserver
{
    ros::NodeHandle nh_;
    //ros::Publisher eyes_pos_pub_;
    ros::Publisher neck_pos_pub_;
    ros::Subscriber keypad_sub_;
    ros::Subscriber reeti_pos_sub_;

    ros::ServiceClient earsClient = nh_.serviceClient<reetiros::MoveEars>("MoveEars");
    ros::ServiceClient sayClient = nh_.serviceClient<reetiros::Say>("SayEnglish");
    ros::ServiceClient neckClient = nh_.serviceClient<reetiros::MoveNeck>("MoveNeck");

    reetiros::MoveEars ears_srv;
    reetiros::Say say_english_srv;
    reetiros::MoveNeck neck_srv;
    
    float servo_deg_yaw = 50;
    float servo_deg_pitch= 50;

    float servo_reeti_yaw = 50;
    float servo_reeti_pitch = 50;

    float servo_reeti_neck_yaw = 50;
    float servo_reeti_neck_pitch = 50;
    float servo_reeti_neck_roll = 50;

public:
    ReetiROSserver()
        {
            neck_pos_pub_ = nh_.advertise<reetiros::reetiNeckPose>("/reeti/neck",1);

            reeti_pos_sub_ = nh_.subscribe("reeti/reetiPose", 1, &ReetiROSserver::reetiPoseCallback, this);
            keypad_sub_ = nh_.subscribe("key", 1, &ReetiROSserver::keyCb, this);
        }
    ~ReetiROSserver()
        {
            
        }

    void keyCb(const std_msgs::Char& keymsg);

    void reetiPoseCallback(const reetiros::reetiPose& msg);

};

void ReetiROSserver::reetiPoseCallback(const reetiros::reetiPose& msg)
{
    servo_reeti_yaw = msg.leftEyeYaw;
    servo_reeti_pitch = msg.leftEyePitch;
    //ROS_INFO("left eye: %f, %f", servo_reeti_yaw, servo_reeti_pitch);

    servo_reeti_neck_yaw = msg.neckYaw;
    servo_reeti_neck_pitch = msg.neckPitch;
    servo_reeti_neck_roll = msg.neckRoll;
    //ROS_INFO("neck: %f, %f, %f", servo_reeti_neck_yaw, servo_reeti_neck_pitch, servo_reeti_neck_roll);

}

void ReetiROSserver::keyCb(const std_msgs::Char& key_msg)
{
    char c;
    bool neck_update = false;

    c = key_msg.data;

    switch(c)
    {
    case KEYCODE_LEFT_ARROW:
        ROS_DEBUG("LEFT");
        neck_update = true;
        servo_reeti_neck_yaw-=1;
        break;
    case KEYCODE_RIGHT_ARROW:
        ROS_DEBUG("RIGHT");
        servo_reeti_neck_yaw+=1;
        neck_update = true;
        break;
    case KEYCODE_UP_ARROW:
        ROS_DEBUG("UP");
        servo_reeti_neck_roll+=1;
        neck_update = true;
        break;
    case KEYCODE_DOWN_ARROW:
        ROS_DEBUG("DOWN");
        servo_reeti_neck_roll-=1;
        neck_update = true;
        break;
    case KEYCODE_a:
        ROS_DEBUG("a");

        break;
    case KEYCODE_d:
        ROS_DEBUG("d");

        break;
    case KEYCODE_e:
        ROS_INFO("e");
 
        ears_srv.request.rightEar = 20;
        ears_srv.request.leftEar = 70;
        if (earsClient.call(ears_srv))
        {
            ROS_INFO("sent");
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
            //return 1;
        }
        break;
    case KEYCODE_h:
        ROS_INFO("h");

        neck_srv.request.neckYaw = servo_reeti_neck_yaw;
        neck_srv.request.neckPitch = servo_reeti_neck_pitch;
        neck_srv.request.neckRoll = servo_reeti_neck_roll + 10;
        neckClient.call(neck_srv);
        
        say_english_srv.request.textToSay = "Hello!" ;
        sayClient.call(say_english_srv);

        ros::Duration(1).sleep();
        
        neck_srv.request.neckYaw = servo_reeti_neck_yaw;
        neck_srv.request.neckPitch = servo_reeti_neck_pitch;
        neck_srv.request.neckRoll = servo_reeti_neck_roll - 10;
        neckClient.call(neck_srv);
        
        break;
    case KEYCODE_r:
        ROS_INFO("r");
 
        ears_srv.request.rightEar = 70;
        ears_srv.request.leftEar = 20;
        if (earsClient.call(ears_srv))
        {
            ROS_INFO("sent");
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
            //return 1;
        }
        break;
    case KEYCODE_f:
        ROS_DEBUG("f");

        break;
    case KEYCODE_v:
        ROS_DEBUG("v");

        break;
    case KEYCODE_u:
        ROS_DEBUG("u");

        break;
    }

    if(neck_update == true)
    {
        reetiros::reetiNeckPose neck_msg;

        neck_msg.neckYaw = servo_reeti_neck_yaw;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = servo_reeti_neck_roll;

        //ROS_INFO("Neck %f %f", neck_msg.neckYaw, neck_msg.neckRoll);
        neck_pos_pub_.publish(neck_msg);

        neck_update = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reeti_server");
    ReetiROSserver rs;

    ros::spin();
    return 0;
}
