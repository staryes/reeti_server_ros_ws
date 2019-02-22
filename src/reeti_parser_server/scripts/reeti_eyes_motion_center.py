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

class reeti_eyes_motion_center:

    rightEyeYaw = 50
    rightEyePitch = 50
    leftEyeYaw = 50
    leftEyePitch = 50

    def __init__(self):
        self.eyes_sub = rospy.Subscriber("/reeti/eyes", reetiEyesPose, self.desiredEyesCb)

    def desiredEyesCb(self, reeti):
        self.rightEyeYaw = reeti.rightEyeYaw
        self.rightEyePitch = reeti.rightEyePitch
        self.leftEyeYaw = reeti.leftEyeYaw
        self.leftEyePitch = reeti.leftEyePitch

    def reeti_eyes_client(self) :
        rospy.wait_for_service('MoveEyesQuickly')
        #print "wait for MoveNeck"
        try:
            move_eyes_quickly = rospy.ServiceProxy('MoveEyesQuickly', MoveEyes )
            #print "connect to service server MoveNeck"
            move_eyes_quickly(self.rightEyeYaw, self.rightEyePitch, self.leftEyeYaw, self.leftEyePitch, 0, 0)
            #print ("yaw=%d  Yaw=%d",yaw,Yaw)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def eyes_pub_spin(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            #rospy.loginfo("wowo")
            self.reeti_eyes_client()
            rate.sleep()

def main():
    remc = reeti_eyes_motion_center()
    rospy.init_node('reeti_eyes_motion_center', anonymous = False)
    try:
        #rospy.spin()
        remc.eyes_pub_spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

