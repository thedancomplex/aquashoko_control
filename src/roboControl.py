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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import aquashoko
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.name)
    aquashoko.aquashoko_set(0,0,data.position[0])
    aquashoko.aquashoko_set(0,1,data.position[1])
    aquashoko.aquashoko_set(0,2,data.position[2])
    aquashoko.aquashoko_set(1,0,data.position[3])
    aquashoko.aquashoko_set(1,1,data.position[4])
    aquashoko.aquashoko_set(1,2,data.position[5])
    aquashoko.aquashoko_set(2,0,data.position[6])
    aquashoko.aquashoko_set(2,1,data.position[7])
    aquashoko.aquashoko_set(2,2,data.position[8])
    aquashoko.aquashoko_set(3,0,data.position[9])
    aquashoko.aquashoko_set(3,1,data.position[10])
    aquashoko.aquashoko_set(3,2,data.position[11])
    aquashoko.aquashoko_put()

def listener():
    aquashoko.aquashoko_init()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('aquashoko_listener', anonymous=False)

    rospy.Subscriber('aquashoko_chatter', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
