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
from std_msgs.msg import Bool

class reeti_neck_motion_center:

    neckYaw = 50
    neckPitch = 50
    neckRoll = 50
    neck_update = False

    blinkSwitch = False
    nodSwitch = False
    
    command_cool_down = False
    cooling_count = 0
    
    def __init__(self):
        self.blink_sub = rospy.Subscriber("/reeti/blink", Bool, self.blinkOnOffCb)
        self.nod_sub = rospy.Subscriber("/reeti/nod", Bool, self.nodOnOffCb)

    def blinkOnOffCb(self, isBlinkOn)
        self.blinkSwitch = isBlinkOn.data

    def nodOnOffCb(self, isNodOn)
        self.nodSwitch = isNodOn.data

    def reeti_blink_client(self) :
        rospy.wait_for_service('RunSequence')
        #print "wait for MoveNeck"
        try:
            blinkingSequence = rospy.ServiceProxy('RunSequence', RunSequence )
            #print "connect to service server MoveNeck"
            blinkingSequence("/")
            #print ("yaw=%d  Yaw=%d",yaw,Yaw)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def command_cooling_count(self):
        if self.command_cool_down == False:
            if self.cooling_count > 7:
                self.command_cool_down = True
                self.cooling_count = 0
            else:
                self.cooling_count = self.cooling_count + 1
                
    def background_motion_spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #rospy.loginfo("wowo")
            self.command_cooling_count()
            if self.blink_Switch == True and self.command_cool_down == True:
                self.reeti_blink_client()
                self.command_cool_down = False
            rate.sleep()

def main():
    rcmc = reeti_conversation_motion_center()
    rospy.init_node('reeti_conversation_motion_center', anonymous = False)
    try:
        #rospy.spin()
        rcmc.background_motion_spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

