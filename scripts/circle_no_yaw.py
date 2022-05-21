#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

def angel2rad(angel):
    return angel/180*math.pi

class MavrosOffboardPosctlTest(MavrosTestCommon):

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.target_pos = PoseStamped()
        # self.target_pos.pose.position.x = 0
        # self.target_pos.pose.position.y = 0
        # self.target_pos.pose.position.z = 1
        self.start_time = 0
        self.cur_time = 0
        self.t = 12
        self.pos_setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.target_pos.header = Header()
        self.target_pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.target_pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.target_pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def settarget(self,event):
        self.cur_time = event.current_real
        # self.cur_time = rospy.Time.now()
        dett = (self.cur_time.to_sec() - self.start_time.to_sec())
        rospy.loginfo("dett:{}".format(dett))
        # print("dett:",dett/1e8)
        self.target_pos.pose.position.x = 1*math.sin(angel2rad(dett*60))
        self.target_pos.pose.position.y = 1*math.cos(angel2rad(dett*60))
        self.target_pos.pose.position.z = 1
        
    # Test method
    def test_posctl(self):
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        # self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        self.start_time = rospy.Time.now()
        self.cur_time = self.start_time
        while(1):
            rospy.Timer(rospy.Duration(nsecs = 1e8), self.settarget, oneshot=False)
            # self.settarget()

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)