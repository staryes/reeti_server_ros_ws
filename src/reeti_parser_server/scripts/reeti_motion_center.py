#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
from reetiros.srv import *
from reetiros.msg import *

class reeti_motion_center:

    neckYaw = 50
    neckPitch = 50
    neckRoll = 50

    cuNeckYaw = 50
    cuNeckPitch = 50
    cuNeckRoll = 50
    
    def __init__(self):
        self.neck_sub = rospy.Subscriber("/reeti/neck", reetiNeckPose, self.desiredNeckCb)
        self.pose_sub = rospy.Subscriber("/reeti/reetiPose", reetiPose, self.currentPoseCb)

    def desiredNeckCb(self, reeti):
        self.neckYaw = reeti.neckYaw
        self.neckPitch = reeti.neckPitch
        self.neckRoll = reeti.neckRoll


    def currentPoseCb(self, reeti):
        self.cuNeckYaw = reeti.neckYaw
        self.cuNeckPitch = reeti.neckPitch
        self.cuNeckRoll = reeti.neckRoll
#        self.reeti_neck_client()

    def reeti_neck_client(self) :
        rospy.wait_for_service('MoveNeck')
        #print "wait for MoveNeck"
        try:
            move_neck = rospy.ServiceProxy('MoveNeck', MoveNeck )
            #print "connect to service server MoveNeck"
            yaw = self.neckYaw - self.cuNeckYaw
            if yaw > 10:
                yaw = 3
            elif yaw > 5:
                yaw = 2
            elif yaw < -10:
                yaw = -3
            elif yaw < -5:
                yaw = -2
            elif yaw > 0:
                yaw = 1
            elif yaw < 0:
                yaw = -1
            else:
                yaw = 0

            Yaw = self.cuNeckYaw + yaw 
            
            move_neck (Yaw, self.neckPitch, self.neckRoll)
            print ("yaw=%d  Yaw=%d",yaw,Yaw)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def neck_pub_spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("wowo")
            self.reeti_neck_client()
            rate.sleep()

def main():
    rmc = reeti_motion_center()
    rospy.init_node('reeti_motion_center', anonymous = False)
    try:
        #rospy.spin()
        rmc.neck_pub_spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
    #neck_pos_pub()
    #get_click_point()
    #reeti_neck_client()
    

