#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"

#include <sstream>

#include <cmath>

// int gX = 0;
// int gY = 0;

// bool xy_resonable(float x, float z)
// {
//     bool resonable = false;

//     if ( x > 2000)
//         return resonable;
//     if ( x < -2000)
//         return resonable;
//     if ( z < 5)
//         return resonable;
//     if ( z > 2000)
//         return resonable;

//     resonable = true;
//     return resonable;
// }

class FaceTracking{
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    // input
    //ros::Subscriber p_sub;
    ros::Subscriber start_sub;
    ros::Subscriber stop_sub;
    ros::Subscriber point_sub;

    //output
    ros::Publisher face_tracking_pub;

    int counter;

    int point_x;
    int point_y;

    int rightDeg;
    int leftDeg;

public:
    FaceTracking()
    //: n
    {

        //subscribe
        start_sub = n.subscribe<std_msgs::Bool>("start", 1, &FaceTracking::startCb, this);

        //p_sub = n.subscribe<std_msgs::UInt16MultiArray>("/goal_pose", 1, &FaceTracking::clickCb, this);


        //publish
        face_tracking_pub = n.advertise<std_msgs::UInt8MultiArray>("eyes", 1);

        point_x = 320; //center of image 640x480
        point_y = 240;

    }

    void init()
    {

        stop_sub = n.subscribe<std_msgs::Bool>("stop", 1, &FaceTracking::stopCb, this);

        point_sub = n.subscribe<std_msgs::UInt16MultiArray>("goal_pose", 1, &FaceTracking::clickCb, this);
     }

    void sleep()
    {
        point_sub.shutdown();
        stop_sub.shutdown();
    }

    ~FaceTracking()
    {
    }

    void startCb(const std_msgs::Bool msg){
        std::cout << "starting person follow" << std::endl;
        if (msg.data) {
            init();
        }
    }

    void stopCb(const std_msgs::Bool msg){
        std::cout << "stopping person follow" << std::endl;
        sleep();
    }

    void clickCb(const std_msgs::UInt16MultiArray msg)
    {
        //int num = msg.data.size();

        point_x = msg.data[0];
        point_y = msg.data[1];

        ROS_INFO("point(%d, %d) ", point_x, point_y);

        if ( point_x > 0)
            leftDeg = point_x * 100 / 640;
        rightDeg = 50;


        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::UInt8MultiArray array;// msg;

        array.data.resize(2);
        array.data[0] = rightDeg;
        array.data[1] = leftDeg;

        ROS_INFO("%d, %d", array.data[0], array.data[2]);

        face_tracking_pub.publish(array);

    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_tracking");
    FaceTracking ft;
    ros::spin();
    return 0;
}

// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */
// void clickCallback(const std_msgs::UInt16MultiArray& msg)
// {
//     int num = msg.data.size();
//     int x, y;

//     x = msg.data[0];
//     y = msg.data[1];

//     ROS_INFO("point(%d, %d) ", x, y);

//     gX = x;
//     gY = y;

// }

// /**
//  * This tutorial demonstrates simple sending of messages over the ROS system.
//  */
// int main(int argc, char **argv)
// {
//   /**
//    * The ros::init() function needs to see argc and argv so that it can perform
//    * any ROS arguments and name remapping that were provided at the command line.
//    * For programmatic remappings you can use a different version of init() which takes
//    * remappings directly, but for most command-line programs, passing argc and argv is
//    * the easiest way to do it.  The third argument to init() is the name of the node.
//    *
//    * You must call one of the versions of ros::init() before using any other
//    * part of the ROS system.
//    */
//   ros::init(argc, argv, "where_to_see");

//   /**
//    * NodeHandle is the main access point to communications with the ROS system.
//    * The first NodeHandle constructed will fully initialize this node, and the last
//    * NodeHandle destructed will close down the node.
//    */
//   ros::NodeHandle n;

//   /**
//    * The subscribe() call is how you tell ROS that you want to receive messages
//    * on a given topic.  This invokes a call to the ROS
//    * master node, which keeps a registry of who is publishing and who
//    * is subscribing.  Messages are passed to a callback function, here
//    * called chatterCallback.  subscribe() returns a Subscriber object that you
//    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//    * object go out of scope, this callback will automatically be unsubscribed from
//    * this topic.
//    *
//    * The second parameter to the subscribe() function is the size of the message
//    * queue.  If messages are arriving faster than they are being processed, this
//    * is the number of messages that will be buffered up before beginning to throw
//    * away the oldest ones.
//    */
//   ros::Subscriber sub = n.subscribe("goal_pose", 1000, clickCallback);

//   ros::spin();


//   /**
//    * The advertise() function is how you tell ROS that you want to
//    * publish on a given topic name. This invokes a call to the ROS
//    * master node, which keeps a registry of who is publishing and who
//    * is subscribing. After this advertise() call is made, the master
//    * node will notify anyone who is trying to subscribe to this topic name,
//    * and they will in turn negotiate a peer-to-peer connection with this
//    * node.  advertise() returns a Publisher object which allows you to
//    * publish messages on that topic through a call to publish().  Once
//    * all copies of the returned Publisher object are destroyed, the topic
//    * will be automatically unadvertised.
//    *
//    * The second parameter to advertise() is the size of the message queue
//    * used for publishing messages.  If messages are published more quickly
//    * than we can send them, the number here specifies how many messages to
//    * buffer up before throwing some away.
//    */
//   ros::Publisher face_tracking_pub = n.advertise<std_msgs::UInt8MultiArray>("eyes", 1);

//   ros::Rate loop_rate(10);
//   ros::spinOnce();

//   /**
//    * A count of how many messages we have sent. This is used to create
//    * a unique string for each message.
//    */
//   int count = 0;

//   int rightDeg = 50;
//   int leftDeg = 50;

//   while (ros::ok())
//   {
//     if ( gX > 0)
//       rightDeg = gX * 100 / 640;
//     leftDeg = 50;


//     /**
//      * This is a message object. You stuff it with data, and then publish it.
//      */
//       std_msgs::UInt8MultiArray array;// msg;

//       //std::stringstream ss;
//       //ss << count%40 + 30;
//       //msg.data = ss.str();

//       //ROS_INFO("%s", msg.data.c_str());

//       array.data.resize(2);
//       array.data[0] = rightDeg;
//       array.data[1] = leftDeg;

//       ROS_INFO("%d, %d", array.data[0], array.data[2]);

//     /**
//      * The publish() function is how you send messages. The parameter
//      * is the message object. The type of this object must agree with the type
//      * given as a template parameter to the advertise<>() call, as was done
//      * in the constructor above.
//      */
//     face_tracking_pub.publish(array);

//     ros::spinOnce();

//     loop_rate.sleep();
//     ++count;
//   }


//   return 0;
// }
