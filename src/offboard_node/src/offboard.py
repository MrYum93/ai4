#!/usr/bin/env python
#/***************************************************************************
# offboard node (ROS) example script
# Copyright (c) 2018, Mathias Madsen <matma14@student.sdu.dk> <mwittenm@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
This script acts as an offboardnode for controlling a plane in ArduPilot simulator as a start

NOT YET: It also acts as a FC for a real fixed-wing drone developed in a master thesis by Mathias and Mark

Revision
2018-10-23 - Mathias Madsen
'''
# topics
mavlink_topic_pub =         '/custom_msg'
mavlink_rc_pub =            '/mavros/rc/in'
mavlink_rc_sub =            '/mavros/rc/in'
mavlink_rc_override_pub =   '/mavros/rc/override'
mavlink_topic_sub =         '/mavros/setpoint_position/global'
mavlink_imu_sub =           '/mavros/imu/data'

# imports
import rospy
import struct
from std_msgs.msg import Int8, Bool
from mavros_msgs.msg import (GlobalPositionTarget, RCIn,
                             OverrideRCIn)
from sensor_msgs.msg import Imu


# defines
update_rate = 4
ROLL = AILERON = 0
PITCH = ELEVATOR = 1
THROTTLE = 2
YAW = RUDDER = 3


class OffboardNode:
    def __init__(self):
        self.global_pos_target = GlobalPositionTarget()
        self.imu = Imu()
        self.rc_in = RCIn()
        self.rc_override = OverrideRCIn()

        # launch node
        rospy.init_node('offboard_node', disable_signals=True)
        self.mavlink_msg_publisher = rospy.Publisher(mavlink_topic_pub,
                                                     Int8, queue_size=0)
        self.mavlink_rc_publisher = rospy.Publisher(mavlink_rc_pub,
                                                    RCIn, queue_size=0)
        self.mavlink_rc_override_publisher = rospy.Publisher(mavlink_rc_pub,
                                                             OverrideRCIn, queue_size=0)
        rospy.Subscriber(mavlink_rc_sub, RCIn,
                         self.on_rc_in)
        rospy.Subscriber(mavlink_topic_sub, GlobalPositionTarget,
                         self.on_global_pos_target)
        rospy.Subscriber(mavlink_imu_sub, Imu,
                         self.on_imu)

        self.rate = rospy.Rate(update_rate)
        rospy.sleep(1)  # wait until everything is running

    def on_rc_in(self, msg):
        self.rc_in = msg

    def on_global_pos_target(self, msg):
        self.global_pos_target.latitude = msg.latitude
        self.global_pos_target.longitude = msg.longtitude
        self.global_pos_target.altitude = msg.altitude

    def on_imu(self, msg):
        self.imu.linear_acceleration.x = msg.linear_acceleration.x
        self.imu.linear_acceleration.y = msg.linear_acceleration.y
        self.imu.linear_acceleration.z = msg.linear_acceleration.z

    def rc_publisher(self):
        rc_control = []
        rc_control.insert(AILERON, 1500)
        rc_control.insert(ELEVATOR, 1300)
        rc_control.insert(THROTTLE, 1800)
        rc_control.insert(RUDDER, 1500)
        self.rc_in.channels = (rc_control[AILERON], rc_control[ELEVATOR],
                               rc_control[THROTTLE], rc_control[RUDDER],
                               1800, 1000, 1000, 1800, 0, 0,
                               0, 0, 0, 0, 0, 0)
        print(self.rc_in)
        self.mavlink_rc_publisher.publish(self.rc_in)

    def rc_override_publisher(self):
        rc_control = []
        rc_control.insert(AILERON, 1500)
        rc_control.insert(ELEVATOR, 1300)
        rc_control.insert(THROTTLE, 1800)
        rc_control.insert(RUDDER, 1500)
        self.rc_override.channels = (rc_control[AILERON], rc_control[ELEVATOR],
                                     rc_control[THROTTLE], rc_control[RUDDER],
                                     0, 0, 0, 0)
        self.mavlink_rc_override_publisher.publish(self.rc_override)

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            print('rc signals is being published')
            # self.rc_publisher()
            self.rc_override_publisher()
            # print('imu: ', self.imu)
            # sleep the defined interval
            self.rate.sleep()

if __name__== '__main__':
    print('### Beginning og code ###')

    obj = OffboardNode()
    obj.run()

    print('### End og code ###')
