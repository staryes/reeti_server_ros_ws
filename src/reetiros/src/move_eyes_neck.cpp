#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

#include "reetiros/MoveEyes.h"

#include <sstream>
#include <fstream>

#include "urbi/uclient.hh"

urbi::UClient *Client = NULL;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void moveNeckEyesCallback(const std_msgs::UInt8MultiArray& msg)
{
    int num = msg.data.size();
    int neckYaw, neckPitch, neckRoll;
    int rightEyeYaw, rightEyePitch, leftEyeYaw, leftEyePitch;

    neckYaw = msg.data[0];
    neckPitch = msg.data[1];
    neckRoll = msg.data[2];

    rightEyeYaw = msg.data[3];
    rightEyePitch = msg.data[4];
    leftEyeYaw = msg.data[5];
    leftEyePitch = msg.data[6];

    ROS_INFO("Move right eye to: %d, left eye to: %d", rightEyeYaw, leftEyeYaw);

    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.neckRotat=" << neckYaw
            << ",Global.servo.neckPan=" << neckPitch
            << ",Global.servo.neckTilt=" << neckRoll
            << ",Global.servo.rightEyePan=" << rightEyeYaw
            << ",Global.servo.rightEyeTilt=" << rightEyePitch
            << ",Global.servo.leftEyePan=" << leftEyeYaw
            << ",Global.servo.leftEyeTilt=" << leftEyePitch
            << ";";
        Client->send(str);
    }
    else
    {
        ROS_WARN("No Urbi Client Found!");
    }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  std::ifstream ipAddressFile;
  ipAddressFile.open("/reetiPrograms/configuration/IP.txt");
  if (!ipAddressFile)
  {
      ROS_ERROR("Can not open Reeti IP adress configuration file. Should be in /reetiPrograms/configuration/IP.txt");
      return 1;
  }
  char IPAddress[256];
  ipAddressFile.getline (IPAddress, 256);
  ROS_INFO("Connexion to REETI on IP address %s", IPAddress);

  Client = new urbi::UClient(IPAddress, 54001);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("neck_eyes", 1, moveNeckEyesCallback);

  //ros::Rate loop_rate(10);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
