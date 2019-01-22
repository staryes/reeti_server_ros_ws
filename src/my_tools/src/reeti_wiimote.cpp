#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "reetiros/AnyCmd.h"
#include "reetiros/reetiNeckPose.h"

#include <sstream>
#include <iostream>

class TeleopReeti
{
public:
    TeleopReeti();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void turnNeck(void);
    
    ros::NodeHandle nh_;

    ros::ServiceClient anycmdClient = nh_.serviceClient<reetiros::AnyCmd>("AnyCmd");
    
    ros::Subscriber joy_sub_;
        ros::Publisher neck_pos_pub_;
    reetiros::AnyCmd anycmd_srv;

    float servo_reeti_neck_yaw = 50;
    float servo_reeti_neck_pitch = 50;
    float servo_reeti_neck_roll = 50;

};

void TeleopReeti::turnNeck(void)
{
    std::stringstream str;
    
    str.str("");
    str  << "Global.servo.neckRotat=" << servo_reeti_neck_yaw << " smooth:0.8s,"
         << "Global.servo.neckPan=" << servo_reeti_neck_pitch << " smooth:0.8s,"
         << "Global.servo.neckTilt=" << servo_reeti_neck_roll << " smooth:0.8s,"
         << ";";

    anycmd_srv.request.cmd = str.str(); anycmdClient.call(anycmd_srv);
}

TeleopReeti::TeleopReeti()
{
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopReeti::joyCallback, this);
    neck_pos_pub_ = nh_.advertise<reetiros::reetiNeckPose>("/reeti/neck",1);
}

void TeleopReeti::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    servo_reeti_neck_yaw = joy->axes[0] * 5 + 50;
    servo_reeti_neck_roll = joy->axes[2] *5 + 50 ;
    //joy->axes[2];

    reetiros::reetiNeckPose neck_msg;

    neck_msg.neckYaw = servo_reeti_neck_yaw;
    neck_msg.neckPitch = 50;
    neck_msg.neckRoll = servo_reeti_neck_roll;

    //ROS_INFO("Neck %f %f", neck_msg.neckYaw, neck_msg.neckRoll);
    neck_pos_pub_.publish(neck_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_reeti");
    TeleopReeti teleop_reeti;

    ros::spin();
    return 0;
}