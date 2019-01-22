#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <reetiros/AnyCmd.h>


class TeleopReeti
{
public:
  TeleopReeti();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
  ros::NodeHandle nh_;

     ros::ServiceClient anycmdClient = nh_.serviceClient<reetiros::AnyCmd>("AnyCmd");

  int linear_, angular_;
  double l_scale_, a_scale_;
 
  ros::Subscriber joy_sub_;

        float servo_reeti_neck_yaw = 50;
    float servo_reeti_neck_pitch = 50;
    float servo_reeti_neck_roll = 50;
  
};

void TeleopReeti::turnNeck(void)
{
    std::stringstream str;


        str.str("");
        str  << "Global.servo.neckRotat=" << servo_reeti_neck_yaw <<  " smooth:0.8s,"
             << "Global.servo.neckPan=" << servo_reeti_neck_pitch << " smooth:0.8s,"
             << "Global.servo.neckTilt=" servo_reeti_neck_roll << " smooth:0.8s,"
             << ";";

        anycmd_srv.request.com = str.str();
        anycmdClient.call(anycmd_srv);
}

TeleopReeti::TeleopReeti()
{

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopReeti::joyCallback, this);

}

void TeleopReeti::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    std_msgs::Float32 vel;

    //vel.angular = a_scale_*joy->axes[angular_];
    vel.data = joy->axes[0];
    vel_pub_.publish(vel);

    joy->axes[0];
    joy->axes[1];
    joy->axes[2];
        
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_reeti");
  TeleopReeti teleop_reeti;

  ros::spin();
}