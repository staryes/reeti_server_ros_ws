#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

#include <sstream>

#include <cmath>


bool xy_resonable(float x, float z)
{
    bool resonable = false;

    if ( x > 2000)
        return resonable;
    if ( x < -2000)
        return resonable;
    if ( z < 5)
        return resonable;
    if ( z > 2000)
        return resonable;

    resonable = true;
    return resonable;
}

float shift_x(float x, float s_x)
{
    if ( x > 500 && s_x > 0)
         s_x = s_x * -1;
    else if (x < -500 && s_x < 0)
         s_x = s_x * -1;

    return s_x;
}

float shift_z(float x, float z, float s_z)
{
        if ( z > 1000 && s_z > 0)
            s_z = s_z * -1;
        else if ( z < 200 && s_z < 0)
            s_z = s_z * -1;

    return s_z;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8MultiArray>("eyes", 1);

  ros::Rate loop_rate(20);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  float x = 0.0;
  float z = 100.0;
  float y = (x+y)*0.5;

  float s_x = 100.0;
  float s_z = 150.0;

  float baseline = 70.0;

  int rightDeg = 50;
  int leftDeg = 50;
  int pitch = 50;

  while (ros::ok())
  {
      s_x = shift_x(x, s_x);
      x = x + s_x;

      if ( x > 500 || x < -500 )
      {
          s_z = shift_z(x, z, s_z);
          z = z + s_z;
      }

      rightDeg = atan2f(x - baseline, z) * 180.0 / 3.14159;
      if ( rightDeg < -30 )
          rightDeg = -30;
      if ( rightDeg > 30 )
          rightDeg = 30;
      rightDeg = rightDeg + 70;

      leftDeg = atan2f(x , z) * 180.0 / 3.14159;
      if ( leftDeg < -30 )
          leftDeg = -30;
      if ( leftDeg > 30 )
          leftDeg = 30;
      leftDeg = leftDeg + 30;

      pitch = atan2f(y, z) * 180 / 3.14159;
      if ( pitch < -90)
        pitch = -90;
      if ( pitch > 90)
        pitch = 90;
      pitch = pitch * 0.5 + 50;

      ROS_INFO("p(%f, %f, %f) deg: (%d, %d),(%d, %d)", x, y, z, rightDeg, pitch, leftDeg, pitch);


    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
      std_msgs::UInt8MultiArray array;// msg;

      //std::stringstream ss;
      //ss << count%40 + 30;
      //msg.data = ss.str();

      //ROS_INFO("%s", msg.data.c_str());

      array.data.resize(4);
      array.data[0] = rightDeg;
      array.data[1] = pitch;
      array.data[2] = leftDeg;
      array.data[3] = pitch;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(array);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
