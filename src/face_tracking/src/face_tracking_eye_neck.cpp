#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <face_tracking/reetiEyesPose.h>
#include <face_tracking/reetiNeckPose.h>
#include <face_tracking/reetiPose.h>

static const std::string OPENCV_WINDOW = "Image window";

class FaceDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher eyes_pos_pub_;
  ros::Publisher neck_pos_pub_;
    ros::Subscriber reeti_pos_sub_;

    //-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
    cv::String face_cascade_name = "/home/shoushan/reeti_server_ros_ws/src/face_tracking/res/haarcascade_frontalface_alt.xml";
    //cv::String left_eye_cascade_name = "res/haarcascade_lefteye_2splits.xml";
    //cv::String right_eye_cascade_name = "res/haarcascade_righteye_2splits.xml";
    cv::CascadeClassifier face_cascade;
    //cv::CascadeClassifier left_eye_cascade;
    //cv::CascadeClassifier right_eye_cascade;
    //cv::String main_window_name = "Capture - Face detection";
    //cv::String face_window_name = "Capture - Face";
    //cv::RNG rng(12345);
    //cv::Mat debugImage;
    cv::Mat skinCrCbHist = cv::Mat::zeros(cv::Size(256, 256), CV_8UC1);

  const static float focalLength = 685.5;
  const static float centerX = 344.559882;
  const static float centerY = 216.258888;

  float servo_deg_yaw = 50;
  float servo_deg_pitch= 50;

  float servo_reeti_yaw = 50;
  float servo_reeti_pitch = 50;

  float servo_reeti_neck_yaw = 50;
  float servo_reeti_neck_pitch = 50;
  float servo_reeti_neck_roll = 50;

  int count = 0;

  float x_deg_shift(int x)
  {
    float d_deg;

    d_deg = (centerX - x) /25; //8 * 3 times slow

    //d_deg = atan(d_deg);
    //d_deg = d_deg * 180 / 3.14159;
    //d_deg = d_deg * 0.6;

    return d_deg;
  }

  float y_deg_shift(int y)
  {
    float d_deg;

    d_deg = (centerY - y) / 25;

    //d_deg = atan(d_deg);
    //d_deg = d_deg * 180 / 3.14159;
    //d_deg = d_deg * 0.6;

    return d_deg;
  }



  void send_gaze_point(int x, int y)
  {
    float d_yaw = x_deg_shift(x);
    float d_pitch = y_deg_shift(y);

    float yaw = d_yaw + servo_reeti_yaw;
    if (yaw < 0)
      yaw = 0.0;
    if (yaw > 100)
      yaw = 100.0;
    servo_deg_yaw = yaw;

    float pitch = d_pitch + servo_reeti_pitch;
    if (pitch < 0)
      pitch = 0.0;
    if (pitch > 100)
      pitch = 100.0;
    servo_deg_pitch = pitch;

    ROS_INFO("dYaw: %f, dPitch: %f", yaw, pitch);

    //servo_reeti_yaw = servo_deg_yaw ;
    //servo_reeti_pitch = servo_deg_pitch ;

    //int d_neck_eyes_yaw = 50 - servo_reeti_yaw;

    //servo_reeti_neck_yaw = 50;
    //servo_reeti_neck_roll = 50;

    //    servo_reeti_neck_yaw = servo_reeti_neck_yaw + (servo_reeti_yaw - 50) * 0.01;
    //    servo_reeti_neck_roll= servo_reeti_neck_roll + (servo_reeti_pitch - 50) * 0.01;

    //    servo_reeti_yaw = servo_reeti_yaw - (servo_reeti_yaw - 50) * 0.1;
    //    servo_reeti_pitch = servo_reeti_pitch - (servo_reeti_pitch - 50) * 0.1;

  }

  void detectAndDisplay( cv::Mat frame)
    {
        std::vector<cv::Rect> faces;
        //cv::Mat frame_gray;

        std::vector<cv::Mat> rgbChannels(3);
        cv::split(frame, rgbChannels);
        cv::Mat frame_gray = rgbChannels[2];

        //cvtColor( frame, frame_gray, CV_BGR2GRAY );
        //equalizeHist( frame_gray, frame_gray );
        //cv::pow(frame_gray, CV_64F, frame_gray);
        //-- Detect faces
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2,
                                       0|CV_HAAR_SCALE_IMAGE|CV_HAAR_FIND_BIGGEST_OBJECT,
                                       cv::Size(150, 150) );
        //  findSkin(debugImage);

        for( int i = 0; i < faces.size(); i++ )
        {
            rectangle(frame, faces[i], 1234);
            ROS_INFO("face(%d, %d), width: %d, height: %d", faces[i].x, faces[i].y, faces[i].width, faces[i].height);
            send_gaze_point(faces[0].x + (faces[0].width * 0.5), faces[0].y + (faces[0].height * 0.5));
        }
        //imshow(main_window_name, debugImage);
        //-- Show what you got
        //if (faces.size() > 0) {
        //  findEyes(frame_gray, faces[0]);  //test turn off eye retangle
    }


    int faceDetectorInit(void)
    {
        // Load the cascades
        if( !face_cascade.load( face_cascade_name ) ){
            ROS_INFO("--(!)Error loading face cascade, please change face_cascade_name in source code.\n");
            return -1;
        }
        ROS_INFO("Got face cascade file!");
        eyes_pos_pub_ = nh_.advertise<face_tracking::reetiEyesPose>("reeti/eyes",1);
        neck_pos_pub_ = nh_.advertise<face_tracking::reetiNeckPose>("reeti/neck",1);
        //        servo_pos_pub = nh_.advertise<std_msgs::UInt8MultiArray>("neck_eyes",1);
        reeti_pos_sub_ = nh_.subscribe("reeti/reetiPose", 1, &FaceDetector::reetiPoseCallback, this);

        return 0;
    }

public:
  FaceDetector()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &FaceDetector::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

    faceDetectorInit();
  }

  ~FaceDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

    void reetiPoseCallback(const face_tracking::reetiPose& msg)
    {
        servo_reeti_yaw = msg.leftEyeYaw;
        servo_reeti_pitch = msg.leftEyePitch;
        //ROS_INFO("left eye: %f, %f", servo_reeti_yaw, servo_reeti_pitch);

        servo_reeti_neck_yaw = msg.neckYaw;
        servo_reeti_neck_pitch = msg.neckPitch;
        servo_reeti_neck_roll = msg.neckRoll;
        // ROS_INFO("neck: %f, %f, %f", servo_reeti_neck_yaw, servo_reeti_neck_pitch, servo_reeti_neck_roll);

            count++;
//  if (count < 40)

//    if (count < 40)
//    {
//    }
    if (count == 5)
    {
        face_tracking::reetiNeckPose neck_msg;

        neck_msg.header.stamp = ros::Time::now();
        neck_msg.header.frame_id = "/world";

        neck_msg.neckYaw = 50;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = 50;

        ROS_INFO("Neck!");

        neck_pos_pub_.publish(neck_msg);
    }
    else if (count == 10)
    {
        count = 0;
        face_tracking::reetiNeckPose neck_msg;

        neck_msg.header.stamp = ros::Time::now();
        neck_msg.header.frame_id = "/world";

        neck_msg.neckYaw = 40;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = 50;

        ROS_INFO("Neck! Back!");

        neck_pos_pub_.publish(neck_msg);
    }
    else
    {
        face_tracking::reetiEyesPose eyes_msg;

        eyes_msg.header.stamp = ros::Time::now();
        eyes_msg.header.frame_id = "/world";

        eyes_msg.rightEyeYaw = servo_deg_yaw + 30;
        eyes_msg.rightEyePitch = servo_deg_pitch;
        eyes_msg.leftEyeYaw = servo_deg_yaw;
        eyes_msg.leftEyePitch = servo_deg_pitch;

        ROS_INFO("rYaw: %f, rPitch: %f", servo_reeti_yaw, servo_reeti_pitch);

        eyes_pos_pub_.publish(eyes_msg);
    }
    }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 50 && cv_ptr->image.cols > 50)
    {
        //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        cv::Mat frame =  cv_ptr->image;
        detectAndDisplay(frame);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

    //Output gaze point
    //std_msgs::UInt8MultiArray array;
    //array.data.resize(7);

    //array.data[0] = servo_reeti_yaw + 30; //right eye pan
    //array.data[1] = servo_reeti_pitch;  //right eye tilt
    //array.data[2] = servo_reeti_yaw; // left eye pan
    //array.data[3] = servo_reeti_pitch; // left eye tilt

    // array.data[0] = servo_reeti_neck_yaw;
    // array.data[1] = servo_reeti_neck_pitch;
    // array.data[2] = servo_reeti_neck_roll;
    // array.data[3] = servo_reeti_yaw + 30;
    // array.data[4] = servo_reeti_pitch;
    // array.data[5] = servo_reeti_yaw;
    // array.data[6] = servo_reeti_pitch;


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detector");
  FaceDetector fd;
  ros::spin();
  return 0;
}