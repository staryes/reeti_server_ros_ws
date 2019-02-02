#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/video/tracking.hpp>

class KFTracking
{
    ros::NodeHandle nh_;
    
    cv::KalmanFilter* kalman;
    cv::Mat* measurement;

    ros::Subscriber measuerment_sub_;
    ros::Publisher estimation_pub_;

    
    int KFTrackingInit(void)
        {
            measurement_sub_ = nh_.subscribe("kf/measurement", 1, &KFTracking::measurementCallback, this);

            estimation_pub_ = nh_.advertise<stdmsgs::float32multiarray>("kft/estimation",1);

            return 0;
        }
}


    
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

    const static float focalLength = 170.5;
    const static float centerX = 79.5;
    const static float centerY = 59.5;

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

            d_deg = (centerX - x) /3; //8 * 3 times slow

            //d_deg = atan(d_deg);
            //d_deg = d_deg * 180 / 3.14159;
            //d_deg = d_deg * 0.6;

            return d_deg;
        }

    float y_deg_shift(int y)
        {
            float d_deg;

            d_deg = (centerY - y) / 3;

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

            for( int i = 0; i < faces.size(); i++ )
            {
                rectangle(frame, faces[i], 1234);
                ROS_INFO("face(%d, %d), width: %d, height: %d", faces[i].x, faces[i].y, faces[i].width, faces[i].height);
                send_left_gaze_point(faces[0].x + (faces[0].width * 0.5), faces[0].y + (faces[0].height * 0.5));
            }
            //imshow(main_window_name, debugImage);
            //-- Show what you got
            //if (faces.size() > 0) {
            //  findEyes(frame_gray, faces[0]);  //test turn off eye retangle
            if ( faces.size() > 0)
            {
                count++;

                if (count > 5 && face_tracking_switch) // every 6/30 s 
                {
                    count = 0;
                    reetiros::reetiNeckPose neck_msg;

                    neck_msg.header.stamp = ros::Time::now();
                    neck_msg.header.frame_id = "/world";

                    float eye_yaw_avg = (reeti_right_eye_yaw + reeti_left_eye_yaw) * 0.5;
                    float eye_pitch_avg = (reeti_right_eye_pitch + reeti_left_eye_pitch) * 0.5;
                
                    if (eye_yaw_avg > 60)
                    servo_reeti_neck_yaw++;
                    else if (eye_yaw_avg < 40)
                    servo_reeti_neck_yaw--;

                    if (servo_reeti_neck_yaw > 100)
                    servo_reeti_neck_yaw = 100;
                    else if (servo_reeti_neck_yaw < 0)
                    servo_reeti_neck_yaw = 0;

                    if (eye_pitch_avg > 60)
                    servo_reeti_neck_roll++;
                    else if (eye_pitch_avg < 40)
                    servo_reeti_neck_roll--;

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

                if (face_tracking_switch)
                eyes_pos_pub_.publish(eyes_msg);

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

            for( int i = 0; i < faces.size(); i++ )
            {
                rectangle(frame, faces[i], 1234);
                ROS_INFO("face(%d, %d), width: %d, height: %d", faces[i].x, faces[i].y, faces[i].width, faces[i].height);
                send_right_gaze_point(faces[0].x + (faces[0].width * 0.5), faces[0].y + (faces[0].height * 0.5));
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
    ros::init(argc, argv, "kalman_tracking");
    KFTracking kft;
    ros::spin();
    return 0;
}
