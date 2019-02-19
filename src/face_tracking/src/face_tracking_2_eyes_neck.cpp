#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

#include <reetiros/reetiEyesPose.h>
#include <reetiros/reetiNeckPose.h>
#include <reetiros/reetiPose.h>

static const std::string OPENCV_WINDOW_left = "Left Image window";
static const std::string OPENCV_WINDOW_right = "Right Image window";

class FaceDetector
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber left_image_sub_;
    image_transport::Subscriber right_image_sub_;
    //image_transport::Publisher image_pub_;
    ros::Publisher eyes_pos_pub_;
    ros::Publisher neck_pos_pub_;
    ros::Subscriber reeti_pos_sub_;
    ros::Subscriber face_tracking_switch_sub_;

    bool face_tracking_switch;
    
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

    // kalman filter
    cv::KalmanFilter* kalman_left;
    cv::Mat* measurement_left;

    cv::KalmanFilter* kalman_right;
    cv::Mat* measurement_right;

    const static float focalLength = 170.5;
    const static float centerX = 79.5;
    const static float centerY = 69.5;

    float desired_right_eye_yaw = 50;
    float desired_right_eye_pitch= 50;

    float desired_left_eye_yaw = 50;
    float desired_left_eye_pitch = 50;
    
    float reeti_right_eye_yaw = 50;
    float reeti_right_eye_pitch = 50;

    float reeti_left_eye_yaw = 50;
    float reeti_left_eye_pitch = 50;
    
    float servo_reeti_neck_yaw = 50;
    float servo_reeti_neck_pitch = 50;
    float servo_reeti_neck_roll = 50;

    int count = 0;

    float x_deg_shift(int x)
        {
            float d_deg;

            d_deg = (centerX - x) / 4;//(2*3); //8 * 3 times slow

            //d_deg = atan(d_deg);
            //d_deg = d_deg * 180 / 3.14159;
            //d_deg = d_deg * 0.6;

            return d_deg;
        }

    float y_deg_shift(int y)
        {
            float d_deg;

            d_deg = (centerY - y) / 4;//(2*3);

            //d_deg = atan(d_deg);
            //d_deg = d_deg * 180 / 3.14159;
            //d_deg = d_deg * 0.6;

            return d_deg;
        }



    void send_left_gaze_point(int x, int y)
        {
            float d_yaw = x_deg_shift(x);
            float d_pitch = y_deg_shift(y);

            float yaw = d_yaw + reeti_left_eye_yaw;
            if (yaw < 0)
            yaw = 0.0;
            if (yaw > 100)
            yaw = 100.0;
            desired_left_eye_yaw = yaw;

            float pitch = d_pitch + reeti_left_eye_pitch;
            if (pitch < 0)
            pitch = 0.0;
            if (pitch > 100)
            pitch = 100.0;
            desired_left_eye_pitch = pitch;

        }

    void send_right_gaze_point(int x, int y)
        {
            float d_yaw = x_deg_shift(x);
            float d_pitch = y_deg_shift(y);

            float yaw = d_yaw + reeti_right_eye_yaw;
            if (yaw < 0)
            yaw = 0.0;
            if (yaw > 100)
            yaw = 100.0;
            desired_right_eye_yaw = yaw;

            float pitch = d_pitch + reeti_right_eye_pitch;
            if (pitch < 0)
            pitch = 0.0;
            if (pitch > 100)
            pitch = 100.0;
            desired_right_eye_pitch = pitch;

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
                                           cv::Size(20, 20) );
            //  findSkin(debugImage);

            //Kalman filter
            //2.kalman_left filter prediction
            cv::Mat prediction = kalman_left->predict();
            cv::Point2f predictPt = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
            
            for( int i = 0; i < faces.size(); i++ )
            {
                rectangle(frame, faces[i], 1234);
                ROS_INFO("face(%d, %d), width: %d, height: %d", faces[i].x, faces[i].y, faces[i].width, faces[i].height);
//send_left_gaze_point(faces[0].x + (faces[0].width * 0.5), faces[0].y + (faces[0].height * 0.5));
                
                //3.update measure
                measurement_left->at<float>(0) = (float)faces[0].x;
                measurement_left->at<float>(1) = (float)faces[0].y;

                //4.update
                kalman_left->correct(*measurement_left);

                cv::Rect face_zero = cv::Rect((int)kalman_left->statePost.at<float>(0), (int)kalman_left->statePost.at<float>(1), faces[0].width, faces[0].height);
                rectangle(frame, face_zero, cv::Scalar(0,200,0));

                
                send_left_gaze_point(face_zero.x + (face_zero.width * 0.5), face_zero.y + (face_zero.height * 0.3));
            }
            //imshow(main_window_name, debugImage);
            //-- Show what you got
            //if (faces.size() > 0) {
            //  findEyes(frame_gray, faces[0]);  //test turn off eye retangle
            if ( faces.size() > 0)
            {
                count++;

                if (count > 10 && face_tracking_switch) // every 10/30 s 
                {
                    count = 0;
                    reetiros::reetiNeckPose neck_msg;

                    neck_msg.header.stamp = ros::Time::now();
                    neck_msg.header.frame_id = "/world";

                    float eye_yaw_avg = (reeti_right_eye_yaw + reeti_left_eye_yaw) * 0.5;
                    float eye_pitch_avg = (reeti_right_eye_pitch + reeti_left_eye_pitch) * 0.5;
                
                    if (eye_yaw_avg > 65)
                    servo_reeti_neck_yaw+=abs(eye_yaw_avg-50)/2;
                    else if (eye_yaw_avg < 35)
                    servo_reeti_neck_yaw-=abs(eye_yaw_avg-50)/2;

                    if (servo_reeti_neck_yaw > 100)
                    servo_reeti_neck_yaw = 100;
                    else if (servo_reeti_neck_yaw < 0)
                    servo_reeti_neck_yaw = 0;

                    if (eye_pitch_avg > 60)
                    servo_reeti_neck_roll+=abs(eye_pitch_avg-50);
                    else if (eye_pitch_avg < 40)
                    servo_reeti_neck_roll-=abs(eye_pitch_avg-50);

                    if (servo_reeti_neck_roll> 100)
                    servo_reeti_neck_roll = 100;
                    else if (servo_reeti_neck_roll < 0)
                    servo_reeti_neck_roll = 0;


                    neck_msg.neckYaw = servo_reeti_neck_yaw;
                    neck_msg.neckPitch = 50;
                    neck_msg.neckRoll = servo_reeti_neck_roll;

                    ROS_INFO("Neck %f %f", neck_msg.neckYaw, neck_msg.neckRoll);
                    
                    neck_pos_pub_.publish(neck_msg);

                    //servo_deg_yaw = (servo_deg_yaw + 40.0) * 0.5;
                    //servo_deg_pitch = (servo_deg_pitch + 50) * 0.5;
                }
                
                reetiros::reetiEyesPose eyes_msg;


                eyes_msg.header.stamp = ros::Time::now();
                eyes_msg.header.frame_id = "/world";


                eyes_msg.rightEyeYaw = desired_right_eye_yaw;
                eyes_msg.rightEyePitch = desired_right_eye_pitch;
                eyes_msg.leftEyeYaw = desired_left_eye_yaw;
                eyes_msg.leftEyePitch = desired_left_eye_pitch;

                //ROS_INFO("rYaw: %f, rPitch: %f", servo_reeti_yaw, servo_reeti_pitch);
                //ROS_INFO("dYaw: %f, dPitch: %f", servo_deg_yaw, servo_deg_pitch);

                if (count%2 == 0 && face_tracking_switch)
                {
                    eyes_pos_pub_.publish(eyes_msg);
                }

            }

        }
    void righteyedetectAndDisplay( cv::Mat frame)
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
                                           cv::Size(20, 20) );
            //  findSkin(debugImage);
            //Kalman filter
            //2.kalman_left filter prediction
            cv::Mat prediction = kalman_right->predict();
            cv::Point2f predictPt = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));

            for( int i = 0; i < faces.size(); i++ )
            {
                rectangle(frame, faces[i], 1234);
                ROS_INFO("face(%d, %d), width: %d, height: %d", faces[i].x, faces[i].y, faces[i].width, faces[i].height);
                //send_right_gaze_point(faces[0].x + (faces[0].width * 0.5), faces[0].y + (faces[0].height * 0.5));

                //3.update measure
                measurement_right->at<float>(0) = (float)faces[0].x;
                measurement_right->at<float>(1) = (float)faces[0].y;

                //4.update
                kalman_right->correct(*measurement_right);

                cv::Rect face_zero = cv::Rect((int)kalman_right->statePost.at<float>(0), (int)kalman_right->statePost.at<float>(1), faces[0].width, faces[0].height);
                rectangle(frame, face_zero, cv::Scalar(0,200,0));

                send_right_gaze_point(face_zero.x + (face_zero.width * 0.5), face_zero.y + (face_zero.height * 0.3));
            }


        }

    int faceDetectorInit(void)
        {
            // Load the cascades
            if( !face_cascade.load( face_cascade_name ) ){
                ROS_INFO("--(!)Error loading face cascade, please change face_cascade_name in source code.\n");
                return -1;
            }
            ROS_INFO("Got face cascade file!");
            eyes_pos_pub_ = nh_.advertise<reetiros::reetiEyesPose>("reeti/eyes",1);
            neck_pos_pub_ = nh_.advertise<reetiros::reetiNeckPose>("reeti/neck",1);

            reeti_pos_sub_ = nh_.subscribe("reeti/reetiPose", 1, &FaceDetector::reetiPoseCallback, this);
            face_tracking_switch_sub_ = nh_.subscribe("track_switch", 1, &FaceDetector::trackingSwitchCb, this);
            face_tracking_switch = false;

            // Kalman Filter config
            // 1.kalman_left filter setup
            const int stateNum = 4;
            const int measureNum = 2;
            kalman_left = new cv::KalmanFilter(stateNum, measureNum ,0); //state_lefg(x,y,deltaX,deltaY)
            //state_left = new cv::Mat(stateNum, 1, CV_32F);
            //processNoise = new cv::Mat(stateNum, 1, CV_32F);
            measurement_left = new cv::Mat(cv::Mat::zeros(measureNum, 1, CV_32F)); //measure(x,y)

            //cv::randn( *state_left, cv::Scalar::all(0), cv::Scalar::all(0.1) );

            kalman_left->transitionMatrix = (cv::Mat_<float>(stateNum, stateNum) <<
                                             // transition matrix
                                             1, 0, 1, 0,
                                             0, 1, 0, 1,
                                             0, 0, 1, 0,
                                             0, 0, 0, 1
                );

            cv::setIdentity(kalman_left->measurementMatrix, cv::Scalar::all(1));
            cv::setIdentity(kalman_left->processNoiseCov, cv::Scalar::all(1e-1));
            cv::setIdentity(kalman_left->measurementNoiseCov, cv::Scalar::all(1e-3));
            cv::setIdentity(kalman_left->errorCovPost, cv::Scalar::all(1));

            //initialize post state_lefg of kalman_left filter at random
            cv::randn(kalman_left->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

            kalman_right = new cv::KalmanFilter(stateNum, measureNum ,0); //state_lefg(x,y,deltaX,deltaY)
            //state_right = new cv::Mat(stateNum, 1, CV_32F);
            //processNoise = new cv::Mat(stateNum, 1, CV_32F);
            measurement_right = new cv::Mat(cv::Mat::zeros(measureNum, 1, CV_32F)); //measure(x,y)

            //cv::randn( *state_right, cv::Scalar::all(0), cv::Scalar::all(0.1) );

            kalman_right->transitionMatrix = (cv::Mat_<float>(stateNum, stateNum) <<
                                              // transition matrix
                                              1, 0, 1, 0,
                                              0, 1, 0, 1,
                                              0, 0, 1, 0,
                                              0, 0, 0, 1
                );

            cv::setIdentity(kalman_right->measurementMatrix, cv::Scalar::all(1));
            cv::setIdentity(kalman_right->processNoiseCov, cv::Scalar::all(1e-1));
            cv::setIdentity(kalman_right->measurementNoiseCov, cv::Scalar::all(1e-3));
            cv::setIdentity(kalman_right->errorCovPost, cv::Scalar::all(1));

            //initialize post state_lefg of kalman_left filter at random
            cv::randn(kalman_right->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

            return 0;
        }

public:
    FaceDetector()
        : it_(nh_)
        {
            // Subscrive to input video feed and publish output video feed
            left_image_sub_ = it_.subscribe("/leftcam/image_raw", 1,
                                            &FaceDetector::leftimageCb, this);
            right_image_sub_ = it_.subscribe("/rightcam/image_raw", 1,
                                             &FaceDetector::rightimageCb, this);
            //image_pub_ = it_.advertise("/image_converter/output_video", 1);

            cv::namedWindow(OPENCV_WINDOW_right);
            cv::namedWindow(OPENCV_WINDOW_left);

            faceDetectorInit();
        }

    ~FaceDetector()
        {
            cv::destroyWindow(OPENCV_WINDOW_right);
            cv::destroyWindow(OPENCV_WINDOW_left);
        }

    void trackingSwitchCb(const std_msgs::Bool& msg)
        {
            face_tracking_switch = msg.data;
        }

    void reetiPoseCallback(const reetiros::reetiPose& msg)
        {
            reeti_left_eye_yaw = msg.leftEyeYaw;
            reeti_left_eye_pitch = msg.leftEyePitch;

            reeti_right_eye_yaw = msg.rightEyeYaw;
            reeti_right_eye_pitch = msg.rightEyePitch;
            //ROS_INFO("left eye: %f, %f", servo_reeti_yaw, servo_reeti_pitch);

            servo_reeti_neck_yaw = msg.neckYaw;
            servo_reeti_neck_pitch = msg.neckPitch;
            servo_reeti_neck_roll = msg.neckRoll;

            //ROS_INFO("neck: %f, %f, %f", servo_reeti_neck_yaw, servo_reeti_neck_pitch, servo_reeti_neck_roll);

        }


    void leftimageCb(const sensor_msgs::ImageConstPtr& msg)
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
            cv::imshow(OPENCV_WINDOW_left, cv_ptr->image);
            cv::waitKey(3);

            // Output modified video stream
            //image_pub_.publish(cv_ptr->toImageMsg());

        }
    void rightimageCb(const sensor_msgs::ImageConstPtr& msg)
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
                righteyedetectAndDisplay(frame);
            }

            // Update GUI Window
            cv::imshow(OPENCV_WINDOW_right, cv_ptr->image);
            cv::waitKey(3);

            // Output modified video stream
            //image_pub_.publish(cv_ptr->toImageMsg());

        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_detector");
    FaceDetector fd;
    ros::spin();
    return 0;
}
