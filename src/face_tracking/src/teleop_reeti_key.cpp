#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

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

class TeleopReeti
{
public:
  TeleopReeti();
  void keyLoop();

 
private:

    ros::NodeHandle nh_;

    ros::Publisher key_pub_;
  
};

TeleopReeti::TeleopReeti()
{
    key_pub_ = nh_.advertise<std_msgs::Char>("key",1);

}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_reeti_key");
  TeleopReeti teleop_reeti;

  signal(SIGINT,quit);

  teleop_reeti.keyLoop();
  
  return(0);
}


void TeleopReeti::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                          
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move Reeti's head");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    ROS_DEBUG("value: 0x%02X\n", c);
    
    switch(c)
    {
      case KEYCODE_LEFT_ARROW:
        ROS_DEBUG("LEFT");
        dirty = true;
        break;
      case KEYCODE_RIGHT_ARROW:
        ROS_DEBUG("RIGHT");
        dirty = true;
        break;
      case KEYCODE_UP_ARROW:
        ROS_DEBUG("UP");
        dirty = true;
        break;
      case KEYCODE_DOWN_ARROW:
        ROS_DEBUG("DOWN");
        dirty = true;
        break;
    case KEYCODE_A:
        ROS_DEBUG("a");
        dirty = true;
        break;
    case KEYCODE_D:
        ROS_DEBUG("d");
        dirty = true;
        break;
    case KEYCODE_F:
        ROS_DEBUG("f");
        dirty = true;
        break;
    case KEYCODE_V:
        ROS_DEBUG("v");
        dirty = true;
        break;
    case KEYCODE_U:
        ROS_DEBUG("u");
        dirty = true;
        break;
    }

    std_msgs::Char key_msg;
    key_msg.data = c;

    if(dirty ==true)
    {
       key_pub_.publish(key_msg);
      dirty=false;
    }
  }


  return;
}



