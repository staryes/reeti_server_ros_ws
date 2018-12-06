#include <ros/ros.h>
#include <std_msgs/Char.h>

#include <face_tracking/reetiEyesPose.h>
#include <face_tracking/reetiNeckPose.h>
#include <face_tracking/reetiPose.h>

#define KEYCODE_RIGHT_ARROW 0x43 
#define KEYCODE_LEFT_ARROW 0x44
#define KEYCODE_UP_ARROW 0x41
#define KEYCODE_DOWN_ARROW 0x42

#define KEYCODE_Q 0x71

#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_F 0x66
#define KEYCODE_V 0x76
#define KEYCODE_U 0x75

class ReetiROSserver
{
  ros::NodeHandle nh_;
    //ros::Publisher eyes_pos_pub_;
  ros::Publisher neck_pos_pub_;
  ros::Subscriber keypad_sub_;
    ros::Subscriber reeti_pos_sub_;
    
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
            neck_pos_pub_ = nh_.advertise<face_tracking::reetiNeckPose>("/reeti/neck",1);

            reeti_pos_sub_ = nh_.subscribe("reeti/reetiPose", 1, &ReetiROSserver::reetiPoseCallback, this);
            keypad_sub_ = nh_.subscribe("key", 1, &ReetiROSserver::keyCb, this);
        }
    ~ReetiROSserver()
        {
            
        }

    void keyCb(const std_msgs::Char& keymsg);

    void reetiPoseCallback(const face_tracking::reetiPose& msg);

};

void ReetiROSserver::reetiPoseCallback(const face_tracking::reetiPose& msg)
    {
        servo_reeti_yaw = msg.leftEyeYaw;
        servo_reeti_pitch = msg.leftEyePitch;
        //ROS_INFO("left eye: %f, %f", servo_reeti_yaw, servo_reeti_pitch);

        servo_reeti_neck_yaw = msg.neckYaw;
        servo_reeti_neck_pitch = msg.neckPitch;
        servo_reeti_neck_roll = msg.neckRoll;
         ROS_INFO("neck: %f, %f, %f", servo_reeti_neck_yaw, servo_reeti_neck_pitch, servo_reeti_neck_roll);

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
    case KEYCODE_A:
        ROS_DEBUG("a");

        break;
    case KEYCODE_D:
        ROS_DEBUG("d");

        break;
    case KEYCODE_F:
        ROS_DEBUG("f");

        break;
    case KEYCODE_V:
        ROS_DEBUG("v");

        break;
    case KEYCODE_U:
        ROS_DEBUG("u");

        break;
    }

    if(neck_update == true)
    {
        face_tracking::reetiNeckPose neck_msg;

        neck_msg.neckYaw = servo_reeti_neck_yaw;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = servo_reeti_neck_roll;

        ROS_INFO("Neck %f %f", neck_msg.neckYaw, neck_msg.neckRoll);
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
