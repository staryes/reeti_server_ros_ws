#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

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


        ROS_INFO("value: 0x%02X\n", c);

        std_msgs::Char key_msg;
        key_msg.data = c;

        key_pub_.publish(key_msg);
    }


    return;
}



