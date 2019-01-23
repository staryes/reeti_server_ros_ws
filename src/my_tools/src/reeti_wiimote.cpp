#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include "reetiros/AnyCmd.h"
#include "reetiros/reetiNeckPose.h"

#include <sstream>
#include <iostream>

class TeleopReeti
{
public:
    TeleopReeti();
    void turnNeck(void);
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    ros::NodeHandle nh_;

    ros::ServiceClient anycmdClient = nh_.serviceClient<reetiros::AnyCmd>("AnyCmd");
    
    ros::Subscriber joy_sub_;
    ros::Publisher neck_pos_pub_;
    ros::Publisher face_tracking_switch_pub_;
    reetiros::AnyCmd anycmd_srv;

    float servo_reeti_neck_yaw = 50;
    float servo_reeti_neck_pitch = 50;
    float servo_reeti_neck_roll = 50;

    bool face_tracking_switch = false;

};

void TeleopReeti::turnNeck(void)
{
    std::stringstream str;
    
    str.str("");
    str  << "Global.servo.neckRotat=" << servo_reeti_neck_yaw << " smooth:0.8s,"
         << "Global.servo.neckPan=" << servo_reeti_neck_pitch << " smooth:0.8s,"
         << "Global.servo.neckTilt=" << servo_reeti_neck_roll << " smooth:0.8s,"
         << ";";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);
}

TeleopReeti::TeleopReeti()
{
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopReeti::joyCallback, this);
    neck_pos_pub_ = nh_.advertise<reetiros::reetiNeckPose>("/reeti/neck",1);
    face_tracking_switch_pub_ = nh_.advertise<std_msgs::Bool>("/track_switch",1);
}

void TeleopReeti::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if (joy->buttons[2] == true)
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