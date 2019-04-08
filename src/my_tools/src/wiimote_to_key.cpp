#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Char.h"

#include <sstream>
#include <iostream>

class Wiimote2Key
{
public:
    Wiimote2Key();
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    ros::NodeHandle nh_;
    
    ros::Subscriber joy_sub_;
    ros::Publisher key_pub_;

};

Wiimote2Key::Wiimote2Key()
{
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Wiimote2Key::joyCallback, this);
    key_pub_ = nh_advertise<std_msgs::Char>("key",1);
}

void TeleopReeti::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if (joy->buttons[0] == true)
    {
        servo_reeti_neck_yaw = joy->axes[0] * 5 + 50;
        servo_reeti_neck_roll = joy->axes[2] *5 + 50 ;

        turnNeck();
    }

    if (joy->buttons[4] == true)
    {
        face_tracking_switch = true;
    }
    if (joy->buttons[5] == true)
    {
        face_tracking_switch = false;
    }

    std_msgs::Bool switch_msg;

    switch_msg.data = face_tracking_switch;

    face_tracking_switch_pub_.publish(switch_msg);
        

//     reetiros::reetiNeckPose neck_msg;

//      neck_msg.neckYaw = servo_reeti_neck_yaw;
//      neck_msg.neckPitch = 50;
//      neck_msg.neckRoll = servo_reeti_neck_roll;

//     // //ROS_INFO("Neck %f %f", neck_msg.neckYaw, neck_msg.neckRoll);
//      neck_pos_pub_.publish(neck_msg);
//
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_reeti");
    TeleopReeti teleop_reeti;

    // ros::Rate loop_rate(10);
    // while (ros::ok())
    // {
    //     teleop_reeti.turnNeck();

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::spin();
    
    return 0;
}