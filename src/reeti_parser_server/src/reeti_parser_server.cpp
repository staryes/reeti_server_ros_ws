#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "ros/console.h"

#include "sound_play/sound_play.h"

#include "reetiros/MoveEars.h"
#include "reetiros/Say.h"
#include "reetiros/MoveNeck.h"
#include "reetiros/AnyCmd.h"

#include "reetiros/reetiEyesPose.h"
#include "reetiros/reetiNeckPose.h"
#include "reetiros/reetiPose.h"

#include <sstream>
#include <iostream>
#include <stdlib.h>

#define KEYCODE_RIGHT_ARROW 0x43 
#define KEYCODE_LEFT_ARROW 0x44
#define KEYCODE_UP_ARROW 0x41
#define KEYCODE_DOWN_ARROW 0x42

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_0 0x30

#define KEYCODE_q 0x71
#define KEYCODE_w 0x77
#define KEYCODE_e 0x65
#define KEYCODE_r 0x72
#define KEYCODE_t 0x74
#define KEYCODE_y 0x79
#define KEYCODE_u 0x75
#define KEYCODE_i 0x69
#define KEYCODE_o 0x6F
#define KEYCODE_p 0x70

#define KEYCODE_a 0x61
#define KEYCODE_s 0x73
#define KEYCODE_d 0x64
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_j 0x6A
#define KEYCODE_k 0x6B
#define KEYCODE_l 0x6C

#define KEYCODE_z 0x7A
#define KEYCODE_x 0x78
#define KEYCODE_c 0x63
#define KEYCODE_v 0x76
#define KEYCODE_b 0x62
#define KEYCODE_n 0x6E
#define KEYCODE_m 0x6D

class ReetiROSserver
{
    ros::NodeHandle nh_;

    ros::Time timeNow;
    
    //ros::Publisher eyes_pos_pub_;
    ros::Publisher neck_pos_pub_;
    ros::Publisher static_image_flag_pub_;
    ros::Publisher face_tracking_switch_pub_;
    ros::Publisher blink_switch_pub_;
    ros::Publisher nod_switch_pub_;
    ros::Publisher robotsound_pub_;

    ros::Subscriber keypad_sub_;
    ros::Subscriber reeti_pos_sub_;

    ros::ServiceClient earsClient = nh_.serviceClient<reetiros::MoveEars>("MoveEars");
    ros::ServiceClient sayClient = nh_.serviceClient<reetiros::Say>("SayEnglish");

    ros::ServiceClient anycmdClient = nh_.serviceClient<reetiros::AnyCmd>("AnyCmd");

    reetiros::MoveEars ears_srv;
    reetiros::Say say_english_srv;

    reetiros::AnyCmd anycmd_srv;
    
    float servo_deg_yaw;
    float servo_deg_pitch;

    float servo_reeti_righteye_yaw;
    float servo_reeti_righteye_pitch;

    float servo_reeti_lefteye_yaw;
    float servo_reeti_lefteye_pitch;

    float servo_reeti_neck_yaw;
    float servo_reeti_neck_pitch;
    float servo_reeti_neck_roll;

    void sequence_say_hello(void);
//    void sequence_greeting(void);
    void sequence_see_monitor(int monior_x);
    void sequence_to_rest_pose(void);
    void sequence_standby(void);
    void sequence_look_down(void);

    void reeti_to_neutral_position(void);
    
    void sequence_exp_1_routine(bool tracking);
    void sequence_exp_1_all_correct(bool tracking);

    void sequence_exp_2_explain_procedure(bool tracking);
    void sequence_exp_2_routine(int exp2_question);
    void sequence_exp_2_random_motion(void);

    void face_tracking_on_off(bool on);
    void blink_motion_on_off(bool on);
    void nod_motion_on_off(bool on);
    
    void eyes_tracking(int dir);

    int rand_num;

    float target_x;
    float target_z;

    int trail_count;
    bool exp1_eye_contact_mode;
    bool exp1_ok_mode;

    enum exp2state {init, Q1, Q2, Q3, Q4, Q5, Q6};
    exp2state e2s = init;

    enum exp2modesequence {m123 = 0, m132, m213, m231, m312, m321};
    exp2modesequence e2mode;

public:
    ReetiROSserver()
        {
            neck_pos_pub_ = nh_.advertise<reetiros::reetiNeckPose>("/reeti/neck",1);
            static_image_flag_pub_ = nh_.advertise<std_msgs::Int8>("/insertFlag",1);
            face_tracking_switch_pub_ = nh_.advertise<std_msgs::Bool>("/track_switch",1);
            blink_switch_pub_ = nh_.advertise<std_msgs::Bool>("/reeti/blink", 1);
            nod_switch_pub_ = nh_.advertise<std_msgs::Bool>("/reeti/nod", 1);
            robotsound_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1);

            reeti_pos_sub_ = nh_.subscribe("reeti/reetiPose", 1, &ReetiROSserver::reetiPoseCallback, this);
            keypad_sub_ = nh_.subscribe("key", 1, &ReetiROSserver::keyCb, this);

            servo_deg_yaw = 50;
            servo_deg_pitch= 50;

            servo_reeti_righteye_yaw = 60;
            servo_reeti_righteye_pitch = 55;

            servo_reeti_lefteye_yaw = 40;
            servo_reeti_lefteye_pitch = 50;

            servo_reeti_neck_yaw = 50;
            servo_reeti_neck_pitch = 50;
            servo_reeti_neck_roll = 50;

            target_x = 0;
            target_z = 50;
        }
    ~ReetiROSserver()
        {

        }

    void keyCb(const std_msgs::Char& keymsg);

    void reetiPoseCallback(const reetiros::reetiPose& msg);
    

};

void ReetiROSserver::eyes_tracking(int dir)
{
    std::stringstream str;
    str.str("");

    int rEye, lEye;

    rEye = servo_reeti_righteye_yaw;
    lEye = servo_reeti_lefteye_yaw;
    
    switch(dir)
    {
    case 1:
        rEye+=2;
        lEye-=2;
        break;
    case 2:
        rEye-=2;
        lEye+=2;
        break;
    case 3:
        rEye-=3;
        lEye-=3;
        break;
    case 4:
        rEye+=3;
        lEye+=3;
        break;
    default:
        break;
    }
    
    str << "Global.servo.rightEyePan=" << rEye << ",Global.servo.leftEyePan=" << lEye << ";" ;
    
    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);
}

void ReetiROSserver::reeti_to_neutral_position(void)
{
    std::stringstream str;
    str.str("");

    str << "Global.servo.neutralPosition;";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);
}

void ReetiROSserver::face_tracking_on_off(bool on)
{
    std_msgs::Bool switch_msg;

    switch_msg.data = on;

    face_tracking_switch_pub_.publish(switch_msg);
}

void ReetiROSserver::nod_motion_on_off(bool on)
{
    std_msgs::Bool switch_msg;

    switch_msg.data = on;

    nod_switch_pub_.publish(switch_msg);
}

void ReetiROSserver::blink_motion_on_off(bool on)
{
    std_msgs::Bool switch_msg;

    switch_msg.data = on;

    blink_switch_pub_.publish(switch_msg);
}

void ReetiROSserver::sequence_to_rest_pose(void)
{
    face_tracking_on_off(false);
    reetiros::reetiNeckPose neck_msg;
    
    std::stringstream str;
    str.str("");
    str << "Global.servo.rightEyeLid=0,Global.servo.leftEyeLid=0;";
    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);

    ros::Duration(0.5).sleep();

    str.str("");
    str << "Global.servo.rightEyeLid=60,Global.servo.leftEyeLid=60,"
        << "Global.servo.rightEyePan=65,Global.servo.leftEyePan=40,"
        << "Global.servo.rightEyeTilt=16,Global.servo.leftEyeTilt=20"
        << ";";
    anycmd_srv.request.cmd = str.str(); 
    anycmdClient.call(anycmd_srv);

    neck_msg.neckYaw = 45;
    neck_msg.neckPitch = 50;
    neck_msg.neckRoll = 10;
    neck_pos_pub_.publish(neck_msg);
        
}

void ReetiROSserver::sequence_look_down(void)
{
    ros::Duration(0.7).sleep();
    
    std::stringstream str;
    str.str("");
    //str << "Global.servo.neckRotat=" << (int)servo_reeti_neck_yaw + dx/2 << " smooth:1s,";
    //str << "Global.servo.neckTilt=" << 10 << " smooth:1s,";

    //str << "Global.servo.rightEyePan=" << (int)servo_reeti_righteye_yaw + d65 << " smooth:1s,";
    //str << "Global.servo.leftEyePan=" << (int)servo_reeti_lefteye_yaw +dx << " smooth:1s,";

    str << "Global.servo.rightEyeTilt=" << 20 << " smooth:1s,";
    str << "Global.servo.leftEyeTilt=" << 20 << " smooth:1s;";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);

    reetiros::reetiNeckPose neck_msg;
    neck_msg.neckYaw = servo_reeti_neck_yaw;
    neck_msg.neckPitch = servo_reeti_neck_pitch;
    neck_msg.neckRoll = 10;
    neck_pos_pub_.publish(neck_msg);
}

void ReetiROSserver::sequence_standby(void)
{
    reetiros::reetiNeckPose neck_msg;
    std::stringstream str;
    str.str("");
    str << "Global.servo.rightEyeLid=100,Global.servo.leftEyeLid=100,"
        << "Global.servo.rightEyePan=65,Global.servo.leftEyePan=35,Global.servo.rightEyeTilt=45,Global.servo.leftEyeTilt=50,"
        //<< "Global.servo.neckRotat=40 smooth:0.3s,"
        //<< "Global.servo.neckPan=50 smooth:0.3s,"
        //<< "Global.servo.neckTilt=50 smooth:0.3s"
        << ";";
    anycmd_srv.request.cmd = str.str(); 
    anycmdClient.call(anycmd_srv);

    servo_reeti_neck_yaw = 45;
    servo_reeti_neck_pitch = 50;
    servo_reeti_neck_roll = 40;
    
    neck_msg.neckYaw = 45;
    neck_msg.neckPitch = 50;
    neck_msg.neckRoll = 40;
    neck_pos_pub_.publish(neck_msg);
}

void ReetiROSserver::sequence_see_monitor(int monitor_x)
{
    std::stringstream str;
    reetiros::reetiNeckPose neck_msg;
        
    if(monitor_x == 0)
    {
        str.str("");
        str  //<< "Global.servo.neckRotat=10 smooth:0.8s,"
             //<< "Global.servo.neckPan=50 smooth:0.8s,"
             //<< "Global.servo.neckTilt=40 smooth:0.8s,"
            << "Global.servo.rightEyePan=45,Global.servo.leftEyePan=20,Global.servo.rightEyeTilt=40 smooth:0.2s,Global.servo.leftEyeTilt=45 smooth:0.2s"
            << ";";

        neck_msg.neckYaw = 20;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = 40;

    }

    else if(monitor_x == 1)
    {
        str.str("");
        str //<< "Global.servo.neckRotat=80 smooth:0.8s,"
            //<< "Global.servo.neckPan=50 smooth:0.8s,"
            //<< "Global.servo.neckTilt=40 smooth:0.8s,"
            << "Global.servo.rightEyePan=85,Global.servo.leftEyePan=60,Global.servo.rightEyeTilt=40 smooth:0.2s,Global.servo.leftEyeTilt=45 smooth:0.2s"
            << ";";

        neck_msg.neckYaw = 80;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = 40;

    }
    else //back to rest pose
    {
        str.str("");
        str << "Global.servo.rightEyeLid=100,Global.servo.leftEyeLid=100,"
            << "Global.servo.rightEyePan=65,Global.servo.leftEyePan=40,Global.servo.rightEyeTilt=20 smooth:0.2s,Global.servo.leftEyeTilt=20 smooth:0.2s,"
            //<< "Global.servo.neckRotat=40 smooth:0.8s,"
            //<< "Global.servo.neckPan=50 smooth:0.8s,"
            //<< "Global.servo.neckTilt=50 smooth:0.8s"
            << ";";

        neck_msg.neckYaw = 45;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = 40;

    }

    anycmd_srv.request.cmd = str.str(); 
    anycmdClient.call(anycmd_srv);
    
    neck_pos_pub_.publish(neck_msg);
    ros::Duration(1).sleep();
}

void ReetiROSserver::sequence_say_hello(void)
{
    face_tracking_on_off(true);
        
    std::stringstream str;
    str.str("");
    str << "Global.tts.say(\"\\\\voice=Kate \\\\language=English \\\\volume=70 Hello! I am Reeti. Nice to meet you.\"),"
        << "Global.servo.changeLedColorRGB( "
        << 0 << ", "
        << 512 << ", "
        << 100 << ", "
        << 100 << ", "
        << 1 << "),"
        << "Global.servo.neckTilt=" << 10 << " smooth:0.3s;";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);
    
    ros::Duration(1).sleep();

    str.str("");
    str << "Global.servo.neckTilt=" << servo_reeti_neck_roll << " smooth:0.3s;";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);

    ros::Duration(3).sleep();

    str.str("");
    str << "Global.servo.changeLedColorRGB( "
        << 0 << ", "
        << 0 << ", "
        << 0 << ", "
        << 0 << ", "
        << 0 << ");";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);

    ros::Duration(1).sleep();
    
//    face_tracking_on_off(false);

}

void ReetiROSserver::sequence_exp_1_routine(bool tracking)
{

    std_msgs::Int8 insertFlag_msg;
    
    ros::Duration(0.5).sleep();
    if (tracking == true)
    {

        sequence_standby();
        
        ROS_INFO("face tracking on");
    }
    else
    {
//        sequence_standby();
        
        ROS_INFO("face tracking off");
    }

    ros::Duration(0.5).sleep();
    face_tracking_on_off(tracking);

    ros::Duration(2).sleep();

    face_tracking_on_off(false);
    
    timeNow = ros::Time::now();
    rand_num = (timeNow.nsec % 2);
    sequence_see_monitor(rand_num);
    ROS_INFO("reeti turn to %d", rand_num);

    ros::Duration(0.5).sleep();
    
    timeNow = ros::Time::now();
    rand_num = (timeNow.nsec % 4) + 1;
    ROS_INFO("rand_num=%d", rand_num);
    insertFlag_msg.data = rand_num;

    static_image_flag_pub_.publish(insertFlag_msg);
    
    face_tracking_on_off(false);
    trail_count--;
}

void ReetiROSserver::sequence_exp_1_all_correct(bool tracking)
{
    std_msgs::Int8 insertFlag_msg;
    
    ros::Duration(0.5).sleep();
    if (tracking == true)
    {

        sequence_standby();
        
        ROS_INFO("face tracking on");
    }
    else
    {
//        sequence_standby();
        
        ROS_INFO("face tracking off");
    }

    ros::Duration(0.5).sleep();
    face_tracking_on_off(tracking);
    
    ros::Duration(2).sleep();

    face_tracking_on_off(false);

    timeNow = ros::Time::now();
    rand_num = (timeNow.nsec % 2);
    sequence_see_monitor(rand_num);
    ROS_INFO("reeti turn to %d", rand_num);

    ros::Duration(0.5).sleep();
    
    timeNow = ros::Time::now();
    rand_num = (rand_num * 2) + (timeNow.nsec % 2) + 1; // reeti is always right
    ROS_INFO("rand_num=%d", rand_num);
    insertFlag_msg.data = rand_num;

    static_image_flag_pub_.publish(insertFlag_msg);

    face_tracking_on_off(false);
    trail_count--;
}

void ReetiROSserver::sequence_exp_2_explain_procedure(bool tracking)
{
    nod_motion_on_off(false);
    blink_motion_on_off(false);
        
    sequence_standby();
    ros::Duration(1).sleep();

    face_tracking_on_off(true);

//    ros::Duration(1).sleep();
    
    std::stringstream str;
    str.str("");
    str //<< "Global.tts.say(\"\\\\voice=Kate \\\\language=English \\\\volume=70 Hello! I am Reeti. Nice to meet you. \"),"
        << "Global.servo.changeLedColorRGB( "
        << 0 << ", "
        << 512 << ", "
        << 300 << ", "
        << 100 << ", "
        << 1 << "),"
        //<< "Global.tts.say(\"\\\\voice=Kate \\\\language=English \\\\volume=70 Are you ready to play a question game? I hope you will have fun. \")"
        << ";"
        ;

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);

    sound_play::SoundRequest soundR;
    soundR.sound = -2;
    soundR.command = 1;
    soundR.volume = 1.0;
    soundR.arg = "/home/shoushan/reeti_server_ros_ws/src/reeti_parser_server/questions_audio/greeting.wav";

    robotsound_pub_.publish(soundR);

    ros::Duration(7).sleep();
    
    str.str("");

        str << "Global.servo.changeLedColorRGB( "
        << 0 << ", "
        << 0 << ", "
        << 0 << ", "
        << 0 << ", "
        << 0 << ")"
        << ";"
        ;

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);
    
    face_tracking_on_off(false);

    timeNow = ros::Time::now();
    e2mode = static_cast<exp2modesequence>(timeNow.nsec % 6) ; 
}

void ReetiROSserver::sequence_exp_2_random_motion(void)
{
    ros::Duration(0.7).sleep();

    timeNow = ros::Time::now();

    srand(timeNow.nsec);
    int dx = rand() % 60 - 30;
    int dy = rand() % 60 - 30;
//    ROS_INFO("dx = %d, dy = %d", dx, dy);

    std::stringstream str;
    str.str("");
    str << "Global.servo.rightEyePan=" << (int)servo_reeti_righteye_yaw + dx << " smooth:0.8s,";
    str << "Global.servo.leftEyePan=" << (int)servo_reeti_lefteye_yaw +dx << " smooth:0.8s,";

    str << "Global.servo.rightEyeTilt=" << (int)servo_reeti_righteye_pitch +dy << " smooth:0.8s,";
    str << "Global.servo.leftEyeTilt=" << (int)servo_reeti_lefteye_pitch + dy << " smooth:0.8s;";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);

    ros::Duration(1).sleep();

    reetiros::reetiNeckPose neck_msg;
    neck_msg.neckYaw = servo_reeti_neck_yaw + dx/2;
    neck_msg.neckPitch = servo_reeti_neck_pitch;
    neck_msg.neckRoll = servo_reeti_neck_roll + dy;
    //neck_pos_pub_.publish(neck_msg);

    str.str("");
    str << "Global.servo.neckRotat=" << (int)servo_reeti_neck_yaw + dx/2 << " smooth:1s,";
    str << "Global.servo.neckTilt=" << (int)servo_reeti_neck_roll + dy << " smooth:1s,";
    
    str << "Global.servo.rightEyePan=" << 60 << " smooth:1s,";
    str << "Global.servo.leftEyePan=" << 40 << " smooth:1s,";

    str << "Global.servo.rightEyeTilt=" << (int)servo_reeti_righteye_pitch - dy/10<< " smooth:1s,";
    str << "Global.servo.leftEyeTilt=" <<  (int)servo_reeti_lefteye_pitch - dy/10 << " smooth:1s;";

    anycmd_srv.request.cmd = str.str();
    anycmdClient.call(anycmd_srv);
    
}

void ReetiROSserver::sequence_exp_2_routine(int exp2_questions)
{
    nod_motion_on_off(false);
    blink_motion_on_off(false);
            
    std::stringstream str;
    str.str("");
    str << "/home/shoushan/reeti_server_ros_ws/src/reeti_parser_server/questions_audio/";

    switch(exp2_questions)
    {
    case 1:
        sequence_standby();
    
        face_tracking_on_off(true);
        //str << "What is your favorite fruit?";
        str << "Q1-1.wav";
        break;
    case 2:
        //str << "Can it be fried?";
        str << "Q1-2.wav";
        break;
    case 20:
//        str << "I believe everything can be fried.";
        str << "Q1-3.wav";
        break;
    case 3:
//        str << "I will ask Shou-Shan to fry it.";
        str << "Q1-4.wav";
        break;
    case 4:
        sequence_standby();
    
        face_tracking_on_off(true);
//        str << "Is tomato a fruit or a vegetable?";
        str << "Q2-1.wav";
        break;
    case 5:
//        str << "Why?";
        str << "Q2-2_why.wav";
        break;
    case 6:
//        str << "It is red, I guess it is meat.";
        str << "Q2-3.wav";
        break;
    case 7:
        sequence_standby();
    
        face_tracking_on_off(true);
//        str << "Are you a dog person or a cat person?";
        str << "Q3-1.wav";
        break;
    case 8:
//        str << "Why?";
        str << "Q3-2_why.wav";
        break;
    case 9:
//        str << "They said that I am a frog robot.";
        str << "Q3-3.wav";
        break;
    case 10:
        sequence_standby();
    
        face_tracking_on_off(true);
//        str << "Have you ever danced in the rain?";
        str << "Q4-1.wav";
        break;
    case 11:
//        str << "Why not?";
        str << "Q4-2.wav";
        break;
    case 12:
//        str << "Why did you do that?";
        str << "Q4-3.wav";
        break;
    case 13:
//        str << "I won't try. I am not waterproof.";
        str << "Q4-4.wav";
        break;
    case 14:
        sequence_standby();
    
        face_tracking_on_off(true);
//        str << "What's the most important holiday in your home country?";
        str << "Q5-1.wav";
        break;
    case 15:
//        str << "Why is it important?";
        str << "Q5-2.wav";
        break;
    case 16:
//        str << "As a robot, I don't have any holiday.";
        str << "Q5-3.wav";
        break;
    case 17:
        sequence_standby();
    
        face_tracking_on_off(true);
//        str << "What's your favorite book?";
        str << "Q6-1.wav";
        break;
    case 18:
//        str << "Tell me more.";
        str << "Q6-2.wav";
        break;
    case 19:
//        str << "The only book I have is my manual.";
        str << "Q6-3.wav";
        break;
            
        
    }
    //str << "\");";
    
    //   anycmd_srv.request.cmd = str.str();
    // anycmdClient.call(anycmd_srv);
    ros::Duration(1).sleep();
    
    sound_play::SoundRequest soundR;
    soundR.sound = -2;
    soundR.command = 1;
    soundR.volume = 1.0;
    soundR.arg = str.str();

    robotsound_pub_.publish(soundR);
    
    ros::Duration(2).sleep();
    
    switch(e2mode)
    {
    case m123:
        if (e2s == Q1 || e2s == Q2)
        {
            face_tracking_on_off(true);
//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q12");
            ROS_INFO("eye contact");
        }
        else if (e2s == Q3 || e2s == Q4)
        {
            face_tracking_on_off(false);//random
            sequence_exp_2_random_motion();
//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q34");
            ROS_INFO("random");
        }
        else
        {
            face_tracking_on_off(false); //look down
            sequence_look_down();
//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q56");
            ROS_INFO("look down");
        }
        break;
    case m132:
        if (e2s == Q1 || e2s == Q2)
        {
            face_tracking_on_off(true);

//            nod_motion_on_off(true);
            blink_motion_on_off(true);

            ROS_INFO("Q12");
            ROS_INFO("eye contact");
        }
        else if (e2s == Q3 || e2s == Q4)
        {
            face_tracking_on_off(false); //look down
            sequence_look_down();
//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q34");
            ROS_INFO("look down");
        }
        else
        {
            face_tracking_on_off(false);//random
            sequence_exp_2_random_motion();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q56");
            ROS_INFO("random");
        }
        break;
    case m213:
        if (e2s == Q1 || e2s == Q2)
        {
            face_tracking_on_off(false); //random
            sequence_exp_2_random_motion();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q12");
            ROS_INFO("random");
        }
        else if (e2s == Q3 || e2s == Q4)
        {
            face_tracking_on_off(true);

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q34");
            ROS_INFO("eye contact");
        }
        else
        {
            face_tracking_on_off(false); //look down
            sequence_look_down();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q56");
            ROS_INFO("look down");
        }
        break;
                
    case m231:
        if (e2s == Q1 || e2s == Q2)
        {
            face_tracking_on_off(false); //random
            sequence_exp_2_random_motion();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q12");
            ROS_INFO("random");
        }
        else if (e2s == Q3 || e2s == Q4)
        {
            face_tracking_on_off(false); //look down
            sequence_look_down();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q34");
            ROS_INFO("look down");
        }
        else
        {
            face_tracking_on_off(true);

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q56");
            ROS_INFO("eye contact");
        }
        break;
    case m312:
        if (e2s == Q1 || e2s == Q2)
        {
            face_tracking_on_off(false); //look down
            sequence_look_down();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q12");
            ROS_INFO("look down");
        }
        else if (e2s == Q3 || e2s == Q4)
        {
            face_tracking_on_off(true);

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q34");
            ROS_INFO("eye contact");
        }
        else
        {
            face_tracking_on_off(false); //random
            sequence_exp_2_random_motion();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q56");
            ROS_INFO("random");
        }
        break;
    case m321:
        if (e2s == Q1 || e2s == Q2)
        {
            face_tracking_on_off(false); //look down
            sequence_look_down();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q12");
            ROS_INFO("look down");
        }
        else if (e2s == Q3 || e2s == Q4)
        {
            face_tracking_on_off(false); //random
            sequence_exp_2_random_motion();

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q34");
            ROS_INFO("random");
        }
        else
        {
            face_tracking_on_off(true);

//            nod_motion_on_off(true);
            blink_motion_on_off(true);
            ROS_INFO("Q56");
            ROS_INFO("eye contact");
        }
        break;
    
    }
    
}

void ReetiROSserver::reetiPoseCallback(const reetiros::reetiPose& msg)
{
    servo_reeti_lefteye_yaw = msg.leftEyeYaw;
    servo_reeti_lefteye_pitch = msg.leftEyePitch;

    servo_reeti_righteye_yaw = msg.rightEyeYaw;
    servo_reeti_righteye_pitch = msg.rightEyePitch;
    //ROS_INFO("left eye: %f, %f", servo_reeti_yaw, servo_reeti_pitch);

    servo_reeti_neck_yaw = msg.neckYaw;
    servo_reeti_neck_pitch = msg.neckPitch;
    servo_reeti_neck_roll = msg.neckRoll;
    //ROS_INFO("neck: %f, %f, %f", servo_reeti_neck_yaw, servo_reeti_neck_pitch, servo_reeti_neck_roll);

}

void ReetiROSserver::keyCb(const std_msgs::Char& key_msg)
{
    char c;
    bool neck_update = false;
    bool send_image_flag = false;

    c = key_msg.data;

    switch(c)
    {
    case KEYCODE_LEFT_ARROW:
        ROS_DEBUG("LEFT");
        neck_update = true;
        servo_reeti_neck_yaw-=1;
        break;
    case KEYCODE_RIGHT_ARROW:
        ROS_DEBUG("RIGHT");
        servo_reeti_neck_yaw+=1;
        neck_update = true;
        break;
    case KEYCODE_UP_ARROW:
        ROS_DEBUG("UP");
        servo_reeti_neck_roll+=1;
        neck_update = true;
        break;
    case KEYCODE_DOWN_ARROW:
        ROS_DEBUG("DOWN");
        servo_reeti_neck_roll-=1;
        neck_update = true;
        break;
    case KEYCODE_a:
        ROS_DEBUG("a");
        sequence_to_rest_pose();
        ROS_INFO("Reeti takes a break");
        break;

    case KEYCODE_c:
        ROS_DEBUG("c");
        ROS_INFO("exp1 ok mode");
        trail_count = 10;
        exp1_ok_mode = true;
        sequence_exp_1_all_correct(true);
        break;
        
    case KEYCODE_d:
        ROS_DEBUG("d");
        ROS_INFO("exp1 d mode");
        trail_count = 10;
        exp1_ok_mode = false;
        exp1_eye_contact_mode = true;
        sequence_exp_1_routine(exp1_eye_contact_mode);

        break;

    case KEYCODE_e:
        ROS_DEBUG("e");
 
        ears_srv.request.rightEar = 20;
        ears_srv.request.leftEar = 70;
        if (earsClient.call(ears_srv))
        {
            ROS_INFO("sent");
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
            //return 1;
        }
        break;
    case KEYCODE_f:
        ROS_DEBUG("f");
        ROS_INFO("exp1 f mode");
        trail_count = 10;
        exp1_ok_mode = false;
        exp1_eye_contact_mode = false;
        sequence_exp_1_routine(exp1_eye_contact_mode);

        break;
    case KEYCODE_h:
        ROS_DEBUG("h");

        sequence_say_hello();
        break;

    case KEYCODE_i:
        eyes_tracking(1);
        break;
    case KEYCODE_j:
        eyes_tracking(3);
        break;
    case KEYCODE_k:
        eyes_tracking(2);
        break;
    case KEYCODE_l:
        eyes_tracking(4);
        break;

        
    case KEYCODE_m:
        ROS_DEBUG("m");
        timeNow = ros::Time::now();
        rand_num = (timeNow.nsec % 4) + 1;
        ROS_INFO("rand_num=%d", rand_num);
        send_image_flag = true;
        break;
    case KEYCODE_n:
        ROS_DEBUG("n");
        timeNow = ros::Time::now();
        rand_num = (timeNow.nsec % 2);
        sequence_see_monitor(rand_num);
        ROS_INFO("reeti turn to %d", rand_num);

        timeNow = ros::Time::now();
        rand_num = (timeNow.nsec % 4) + 1;
        ROS_INFO("rand_num=%d", rand_num);
        send_image_flag = true;

        face_tracking_on_off(false);
        
        break;
    case KEYCODE_o:
        sequence_exp_2_explain_procedure(true);
        break;
    case KEYCODE_p:
        reeti_to_neutral_position();
        break;
    case KEYCODE_q:
        ROS_DEBUG("q");
        sequence_see_monitor(0);
        break;
    case KEYCODE_r:
        ROS_DEBUG("r");
        sequence_exp_2_random_motion();
        // ears_srv.request.rightEar = 70;
        // ears_srv.request.leftEar = 20;
        // if (earsClient.call(ears_srv))
        // {
        //     ROS_INFO("sent");
        // }
        // else
        // {
        //     ROS_ERROR("Failed to call service add_two_ints");
        //     //return 1;
        // }
        break;
    case KEYCODE_s:
        ROS_DEBUG("s");
        sequence_standby();
        ROS_INFO("Reeti standby");
        break;
        
    case KEYCODE_u:
        ROS_DEBUG("u");
        ROS_INFO("user pressed u");
        sequence_see_monitor(-1);
        if(trail_count>0)
        {
            if (exp1_ok_mode)
            sequence_exp_1_all_correct(true);
            else
            sequence_exp_1_routine(exp1_eye_contact_mode);
        }
        else if(trail_count == 0)
        {
            sequence_to_rest_pose();
        }
        break;
    case KEYCODE_v:
        ROS_DEBUG("v");
        ROS_INFO("user pressed v");
        sequence_see_monitor(-1);
        if(trail_count>0)
        {
            if (exp1_ok_mode)
            sequence_exp_1_all_correct(true);
            else
            sequence_exp_1_routine(exp1_eye_contact_mode);
        }
        else if(trail_count == 0)
        {
            sequence_to_rest_pose();
        }
        break;


    case KEYCODE_w:
        ROS_DEBUG("w");
        sequence_see_monitor(1);
        break;
    case KEYCODE_x:
        ROS_DEBUG("x");
        face_tracking_on_off(false);

        break;
    case KEYCODE_z:
        ROS_DEBUG("z");
        face_tracking_on_off(true);


        //sequence_standby();

        break;

    case KEYCODE_1:
        switch (e2s)
        {
        case init:
            //ask question 1
            sequence_exp_2_explain_procedure(true);
            e2s = Q1;
            break;
        case Q1:
            sequence_exp_2_routine(1);
            break;
        case Q2:
            sequence_exp_2_routine(4);
            break;
        case Q3:
            sequence_exp_2_routine(7);
            break;
        case Q4:
            sequence_exp_2_routine(10);
            break;
        case Q5:
            sequence_exp_2_routine(14);
            break;
        case Q6:
            sequence_exp_2_routine(17);
            break;
        }
        break;
    case KEYCODE_2:
        switch (e2s)
        {
        case init:
            //ask question 2
            //e2s = Q2;
            break;
        case Q1:
            sequence_exp_2_routine(2);
            break;

        case Q2:
            sequence_exp_2_routine(5);
                        
            break;

        case Q3:
            sequence_exp_2_routine(8);
            break;

        case Q4:
            sequence_exp_2_routine(11);
            break;

        case Q5:
            sequence_exp_2_routine(15);
            break;

        case Q6:
            sequence_exp_2_routine(18);
            break;
        }
        break;
    case KEYCODE_3:
        switch (e2s)
        {
        case init:
            //ask question 3
            //e2s = Q3;
            break;
        case Q1:
            sequence_exp_2_routine(20);
            break;

        case Q2:
            sequence_exp_2_routine(6);
            e2s = Q3;
            break;

        case Q3:
            sequence_exp_2_routine(9);
            e2s = Q4;
            break;

        case Q4:
            sequence_exp_2_routine(12);
            break;

        case Q5:
            sequence_exp_2_routine(16);
            e2s = Q6;
            break;

        case Q6:
            sequence_exp_2_routine(19);
            e2s = init;
            break;
        }
        break;
    case KEYCODE_4:
        switch(e2s)
        {
        case Q1:
            sequence_exp_2_routine(3);
            e2s = Q2;
            break;
        case Q4:
            sequence_exp_2_routine(13);
            e2s = Q5;
            break;
        }
        break;
    case KEYCODE_0:
        nod_motion_on_off(false);
        blink_motion_on_off(false);
        break;
    }

    if(neck_update == true)
    {
        reetiros::reetiNeckPose neck_msg;

        neck_msg.neckYaw = servo_reeti_neck_yaw;
        neck_msg.neckPitch = 50;
        neck_msg.neckRoll = servo_reeti_neck_roll;

        //ROS_INFO("Neck %f %f", neck_msg.neckYaw, neck_msg.neckRoll);
        neck_pos_pub_.publish(neck_msg);

        neck_update = false;
    }
    
    if(send_image_flag == true)
    {
        std_msgs::Int8 insertFlag_msg;
        insertFlag_msg.data = rand_num;

        static_image_flag_pub_.publish(insertFlag_msg);
        
        send_image_flag = false;
    }

    // sequence_exp_2_random_motion(); //test

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reeti_server");
    ReetiROSserver rs;

    ros::spin();
    return 0;
}
