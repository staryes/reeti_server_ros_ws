#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class FaceDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

    //-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
    cv::String face_cascade_name = "res/haarcascade_frontalface_alt.xml";
    //cv::String left_eye_cascade_name = "res/haarcascade_lefteye_2splits.xml";
    //cv::String right_eye_cascade_name = "res/haarcascade_righteye_2splits.xml";
    cv::CascadeClassifier face_cascade;
    //cv::CascadeClassifier left_eye_cascade;
    //cv::CascadeClassifier right_eye_cascade;
    //cv::String main_window_name = "Capture - Face detection";
    //cv::String face_window_name = "Capture - Face";
    cv::RNG rng(12345);
    //cv::Mat debugImage;
    cv::Mat skinCrCbHist = cv::Mat::zeros(cv::Size(256, 256), CV_8UC1);

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
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE|CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(150, 150) );
        //  findSkin(debugImage);

        for( int i = 0; i < faces.size(); i++ )
        {
            rectangle(debugImage, faces[i], 1234);
        }
        //imshow(main_window_name, debugImage);
        //-- Show what you got
        //if (faces.size() > 0) {
        //  findEyes(frame_gray, faces[0]);  //test turn off eye retangle
        }
    }

    int faceDetectorInit(void)
    {
        // Load the cascades
        if( !face_cascade.load( face_cascade_name ) ){
            ROS_INFO("--(!)Error loading face cascade, please change face_cascade_name in source code.\n");
            return -1;
        }
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
  }

  ~FaceDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::FaceDetector& msg)
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
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
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
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detector");
  FaceDetector fd;
  ros::spin();
  return 0;
}
