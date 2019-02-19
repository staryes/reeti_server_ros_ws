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

class reeti_neck_motion_center:

    neckYaw = 50
    neckPitch = 50
    neckRoll = 50
    neck_update = False

    face_tracking_switch = False
    
    def __init__(self):
        self.neck_sub = rospy.Subscriber("/reeti/neck", reetiNeckPose, self.desiredNeckCb)
 
    def desiredNeckCb(self, reeti):
        self.neckYaw = reeti.neckYaw
        self.neckPitch = reeti.neckPitch
        self.neckRoll = reeti.neckRoll
        self.neck_update = True

    def reeti_neck_client(self) :
        rospy.wait_for_service('MoveNeckSmoothly')
        #print "wait for MoveNeck"
        self.neck_update = False
        try:
            move_neck_smoothly = rospy.ServiceProxy('MoveNeckSmoothly', MoveNeck )
            #print "connect to service server MoveNeck"
            move_neck_smoothly(self.neckYaw, self.neckPitch, self.neckRoll)
            #print ("yaw=%d  Yaw=%d",yaw,Yaw)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def neck_pub_spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            #rospy.loginfo("wowo")
            if self.neck_update == True:
                self.reeti_neck_client()
            rate.sleep()

def main():
    rnmc = reeti_neck_motion_center()
    rospy.init_node('reeti_neck_motion_center', anonymous = False)
    try:
        #rospy.spin()
        rnmc.neck_pub_spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

