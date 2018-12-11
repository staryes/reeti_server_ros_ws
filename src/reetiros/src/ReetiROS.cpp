#include "ros/ros.h"
#include "std_msgs/String.h"
#include "reetiros/SetPose.h"
#include "reetiros/MoveNeck.h"
#include "reetiros/MoveMouth.h"
#include "reetiros/MoveEyes.h"
#include "reetiros/MoveEars.h"
#include "reetiros/SetLed.h"
#include "reetiros/SayAndMoveLips.h"
#include "reetiros/Say.h"
#include "reetiros/RunSequence.h"
#include "reetiros/AnyCmd.h"
#include "reetiros/GetPosition.h"
#include <sstream>
#include <iostream>
#include <fstream>

#include <sys/socket.h>
#include <netinet/in.h>

#include "urbi/uclient.hh"

urbi::UClient *Client = NULL;
/*!
  SetPose set the 15 motors positions
  SetPose  neckYaw, neckPitch, neckRoll,
           mouthRightCorner, mouthLeftCorner,
           topLip, bottomLip,
           rightEyeYaw, rightEyePitch
           leftEyeYaw, leftEyePitch,
           rightEyeLid, leftEyeLid,
           rifhtEar, leftEar

  Values are float between 0 and 100

  */
bool SetPose(reetiros::SetPose::Request& _req,
             reetiros::SetPose::Response& _res )
{
    ROS_INFO("Moving Reeti to requested Pose");
    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.neckRotat=" << _req.pose.neckYaw
            << ",Global.servo.neckPan=" << _req.pose.neckPitch
            << ",Global.servo.neckTilt=" << _req.pose.neckRoll
            << ",Global.servo.rightLC=" << _req.pose.mouthRightCorner
            << ",Global.servo.leftLC=" << _req.pose.mouthLeftCorner
            << ",Global.servo.topLip=" << _req.pose.topLip
            << ",Global.servo.bottomLip=" << _req.pose.bottomLip
            << ",Global.servo.rightEyePan=" << _req.pose.rightEyeYaw
            << ",Global.servo.rightEyeTilt=" << _req.pose.rightEyePitch
            << ",Global.servo.leftEyePan=" << _req.pose.leftEyeYaw
            << ",Global.servo.leftEyeTilt=" << _req.pose.leftEyePitch
            << ",Global.servo.rightEyeLid=" << _req.pose.rightEyeLid
            << ",Global.servo.leftEyeLid=" << _req.pose.leftEyeLid
            << ",Global.servo.rightEar=" << _req.pose.rightEar
            << ",Global.servo.leftEar=" << _req.pose.leftEar
            << ";";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
} 


/*!
  MoveNeck set the 3 motors positions of the neck
  MoveNeck neckYaw, neckPitch, neckRoll

  Values are float between 0 and 100
*/
bool MoveNeck(reetiros::MoveNeck::Request& _req,
              reetiros::MoveNeck::Response& _res )
{
    ROS_INFO("Moving Reeti Neck");
    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.neckRotat=" << _req.neckYaw
            << ",Global.servo.neckPan=" << _req.neckPitch
            << ",Global.servo.neckTilt=" << _req.neckRoll
            << ";";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  MoveMouth set the 4 motors positions of the mouth
  MoveMouth  mouthRightCorner, mouthLeftCorner,
             topLip, bottomLip

  Values are float between 0 and 100
*/
bool MoveMouth(reetiros::MoveMouth::Request& _req,
               reetiros::MoveMouth::Response& _res )
{
    ROS_INFO("Moving Reeti Mouth");
    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.rightLC=" << _req.mouthRightCorner
            << ",Global.servo.leftLC=" << _req.mouthLeftCorner
            << ",Global.servo.topLip=" << _req.topLip
            << ",Global.servo.bottomLip=" << _req.bottomLip
            << ";";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}


/*!
  MoveEyes set the 6 motors positions of the two eyes and two lids
  MoveEyes rightEyeYaw, rightEyePitch
           leftEyeYaw, leftEyePitch,
           rightEyeLid, leftEyeLid

  Values are float between 0 and 100
*/
bool MoveEyes(reetiros::MoveEyes::Request& _req,
              reetiros::MoveEyes::Response& _res )
{
    ROS_INFO("Moving Reeti Eyes");
    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.rightEyePan=" << _req.rightEyeYaw
            << ",Global.servo.rightEyeTilt=" << _req.rightEyePitch
            << ",Global.servo.leftEyePan=" << _req.leftEyeYaw
            << ",Global.servo.leftEyeTilt=" << _req.leftEyePitch
            << ",Global.servo.rightEyeLid=" << _req.rightEyeLid
            << ",Global.servo.leftEyeLid=" << _req.leftEyeLid
            << ";";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  MoveEars set the 2 motors positions of the two ears
  MoveEars rightEar, leftEar

  Values are float between 0 and 100
*/
bool MoveEars(reetiros::MoveEars::Request& _req,
              reetiros::MoveEars::Response& _res )
{
    ROS_INFO("Moving Reeti Ears");
    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.rightEar=" << _req.rightEar
            << ",Global.servo.leftEar=" << _req.leftEar
            << ";";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
Change Led color with a specific color and intensity.

Parameters:
    ledIndex,:	0 = All Led, 1 = Led1 , 2 = Led2
    _red,:	Red value ( between [0;1023])
    _green,:	Green value ( between [0;1023])
    _blue,:	Blue value ( between [0;1023])
    intensity,:	light intensity. Value must be between 0 and 1.

*/
bool SetLed( reetiros::SetLed::Request& _req,
             reetiros::SetLed::Response& _res )
{
    ROS_INFO("Change Reeti Cheeks Colors");
    if(Client)
    {
        std::stringstream str;
        str << "Global.servo.changeLedColorRGB( "
            << _req.ledIndex << ", "
            << _req.red << ", "
            << _req.green << ", "
            << _req.blue << ", "
            << _req.intensity << ");";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  Say a text
  Say textToSay
*/
bool Say(reetiros::Say::Request& _req,
         reetiros::Say::Response& _res )
{
    ROS_INFO("Say a text");
    if(Client)
    {
        std::stringstream str;
        str << "Global.tts.say(\" "
            << _req.textToSay << "\");";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  Say a text
  Say textToSay
*/
bool SayEnglish(reetiros::Say::Request& _req,
         reetiros::Say::Response& _res )
{
    ROS_INFO("Say a text");
    if(Client)
    {
        std::stringstream str;
        str << "Global.tts.say(\"\\\\voice=Simon \\\\language=English "
            << _req.textToSay << "\");";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  Say a text and move Lips
  SayAndMoveLips texToSay
*/
bool SayAndMoveLips(reetiros::SayAndMoveLips::Request& _req,
                    reetiros::SayAndMoveLips::Response& _res )
{
    ROS_INFO("Say And Move Lips");
    if(Client)
    {
        std::stringstream str;
        str << "Global.tts.sayWithSynchro(\" "
            << _req.textToSay << "\");";
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  run a sequence described by its file name from the directory /home/reeti/reetiDocuments/Sequences/
  sequence file name include the path relative to the Sequence directory

  RunSequence("ReetiSeq/accueil") will play the file "/home/reeti/reetiDocuments/Sequences/ReetiSeq/accueil.rmdl"
*/
bool RunSequence(reetiros::RunSequence::Request& _req,
                 reetiros::RunSequence::Response& _res )
{
    ROS_INFO("Run a Sequence");
    if(Client)
    {
        std::stringstream str;
        str << "Global.player.playSequence(\""
            << _req.sequenceName << "\");";
        //ROS_INFO("cmde envoyee : %s", str.str().c_str());
        Client->send(str);
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  MoveMouth set the 4 motors positions of the mouth
  MoveMouth  mouthRightCorner, mouthLeftCorner,
             topLip, bottomLip

  Values are float between 0 and 100
*/
bool AnyCmd(reetiros::AnyCmd::Request& _req,
            reetiros::AnyCmd::Response& _res )
{
    ROS_INFO("Do what you want to do!");
    if(Client)
    {
        std::stringstream str;
        str << _req ;

        //std::stringstream res;
        Client->send(str);

        //_res << res;
        //double a = -1;
        //Client->syncGetDevice("Global.servo.position());",a);
        //SyncClient->syncGetDevice("rightEyePan", a );
        //ROS_INFO("servo positions = %f", a);
        //        _res << str;
        
    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

/*!
  MoveMouth set the 4 motors positions of the mouth
  MoveMouth  mouthRightCorner, mouthLeftCorner,
  topLip, bottomLip

  Values are float between 0 and 100
*/
bool GetPosition(reetiros::GetPosition::Request& _req,
            reetiros::GetPosition::Response& _res )
{
    ROS_INFO("Get Positions");
    if(Client)
    {
        std::stringstream str;
        //str << _req ;

        //std::stringstream res;
        Client->send("Global.servo.position();");

        //Client->processRecvBuffer();

        //Client << "Global.servo.leftEar=" << 20 << ";";

        //str = Client->recvBuffer;

        //_res = "wo";
        // double a = -1;
        //Client->syncGetDevice("Global.servo.position();",a);
        //SyncClient->syncGetDevice("rightEyePan", a );
        //ROS_INFO(_res);
        //        _res << str;

    }else
    {
        ROS_WARN("No Urbi Client found");
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ReetiRosAPI");
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

    ros::ServiceServer SetPoseService = n.advertiseService("SetPose", SetPose);
    ros::ServiceServer MoveNeckService = n.advertiseService("MoveNeck", MoveNeck);
    ros::ServiceServer MoveMouthService = n.advertiseService("MoveMouth", MoveMouth);
    ros::ServiceServer MoveEyesService = n.advertiseService("MoveEyes", MoveEyes);
    ros::ServiceServer MoveEarsService = n.advertiseService("MoveEars", MoveEars);
    ros::ServiceServer SetLedService = n.advertiseService("SetLed", SetLed);
    ros::ServiceServer SayAndMoveLipsService = n.advertiseService("SayAndMoveLips", SayAndMoveLips);
    ros::ServiceServer SayService = n.advertiseService("Say", Say);
    ros::ServiceServer SayEnglishService = n.advertiseService("SayEnglish", SayEnglish);
    ros::ServiceServer RunSequenceService = n.advertiseService("RunSequence", RunSequence);

    ros::ServiceServer AnyCmdService = n.advertiseService("AnyCmd", AnyCmd);
    ros::ServiceServer GetPositionService = n.advertiseService("GetPosition", GetPosition);
    
    ros::spin();

    return 0;
}
