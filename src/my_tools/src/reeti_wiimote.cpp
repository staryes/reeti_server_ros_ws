#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>


class TeleopReeti
{
public:
  TeleopReeti();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopReeti::TeleopReeti():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<std_msgs::Float32>("linear_scalar", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopReeti::joyCallback, this);

}

void TeleopReeti::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    std_msgs::Float32 vel;

    //vel.angular = a_scale_*joy->axes[angular_];
    vel.data = joy->axes[0];
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_reeti");
  TeleopReeti teleop_reeti;

  ros::spin();
}