#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
//#include <iostream>
//#include <fstream>

//#include <stdio.h>
#include <sys/socket.h>
//#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// Client side C/C++ program to demonstrate Socket programming
#define PORT 54001

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
    ros::init(argc, argv, "position");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * TCP/IP socketpair
     */
	struct sockaddr_in address;
	int sock = 0, valread;
	struct sockaddr_in serv_addr;
    char cmd[25] = "Global.servo.position();";
	char buffer[1024] = {0};
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, "192.168.0.100", &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}




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
  ros::Publisher pose_pub = n.advertise<std_msgs::Float32MultiArray>("Position", 1000);

  ros::Rate loop_rate(100);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
      send(sock , cmd , strlen(cmd) , 0 );
      //ROS_INFO("Position message sent\n");
      valread = read( sock , buffer, 1024);
      //ROS_INFO("%s\n",buffer);
      //ROS_INFO_STREAM(buffer );
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

      //example:
      // data: "[00076316] \"50.00a55.56b50.00c43.86d56.14e50.00f80.00g80.00h90.84i42.55j60.00k90.24l40.00m42.55n81.82&stop&&\"\

      std_msgs::Float32MultiArray array;

      array.data.resize(15);

      int stop = 0;
      int n = 0;
      int v = 0;
      int array_p = 0;
      while (stop == 0 )
      {
          char ch = buffer[n];
          n++;
          switch(ch) {
          case '"':
              v = 0;
              break;
          case '0'...'9':
              v = v * 10 + ch - '0';
              break;
          case 'a'...'o':
              array_p = ch - 'a';
              array.data[array_p] = v / 100;
              v = 0;
              break;

          case 'p':
              stop = 1;
              break;
          default:
              break;
          }
      }

      //std_msgs::String msg;

      //std::stringstream ss;
      //ss << buffer;
      //msg.data = ss.str();

      //ROS_INFO("%s", msg.data.c_str());


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pose_pub.publish(array);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }



	return 0;
}
