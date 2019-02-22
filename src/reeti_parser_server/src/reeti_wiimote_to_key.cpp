#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Char.h"

#define KEYCODE_u 0x75
#define KEYCODE_v 0x76

class ReetiWiimote2Key
{
public:
    ReetiWiimote2Key();

    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    ros::NodeHandle nh_;
    
    ros::Subscriber joy_sub_;
    ros::Publisher key_pub_;

    bool wiimoteButtonPressed;

};

ReetiWiimote2Key::ReetiWiimote2Key()
{
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &ReetiWiimote2Key::joyCallback, this);

    key_pub_ = nh_.advertise<std_msgs::Char>("key",1);

    wiimoteButtonPressed = false;

}

void ReetiWiimote2Key::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    std_msgs::Char key_msg;

    if (joy->buttons[0] == true && joy->buttons[1] == false && wiimoteButtonPressed == false)
    {
        key_msg.data  = KEYCODE_v;
        key_pub_.publish(key_msg);
        
        wiimoteButtonPressed = true;


    }
    else if (joy->buttons[0] == false && joy->buttons[1] == true && wiimoteButtonPressed == false)
    {
        key_msg.data = KEYCODE_u;
        key_pub_.publish(key_msg);

        wiimoteButtonPressed = true;
    }
    else if (joy->buttons[0] == false && joy->buttons[1] == false)
    {
        wiimoteButtonPressed = false;
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "reeti_wiimote_to_key");
    ReetiWiimote2Key ReetiWiimote2Key;

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