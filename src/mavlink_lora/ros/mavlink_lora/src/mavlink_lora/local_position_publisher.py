#!/usr/bin/env python
#/***************************************************************************
# Obtain local setpoints from Mavlink and publish to ros topic
#
# Copyright (c) 2018
# Per Breum <lillep3r@gmail.com>
# Mads Tilgaard Jensen <tilgaard1@hotmail.com>
# Mathias Witten Madsen <mwittenm>@gmail.com>
# Blazej Banaszewski <blazej@bananzeweski.pl>
# Oscar Schofield <oschofield95@gmail.com>
#
# Developed in RMUASD course at SDU UAS Center
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
Receives LOCAL_POSITION_NED (MSG #32) using mavlink_lora 
and publish information in topic /mavlink_local_position_ned

'''

#msg ids
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_local_ned_pos_pub = '/mavlink_local_position_ned'
update_interval = 10

# imports
import rospy
import struct
from mavlink_lora.msg import (mavlink_lora_msg, mavlink_lora_local_ned_pos)

class LocalPositionPublisher:
    def __init__(self):
        self.msg = mavlink_lora_msg()
        self.local_pos_msg = mavlink_lora_local_ned_pos()

        # status variables

        # launch node 
        rospy.init_node('local_position_publisher', disable_signals = True)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, 
                         self.on_mavlink_msg)
        self.local_ned_pub = rospy.Publisher(mavlink_lora_local_ned_pos_pub, 
                                             mavlink_lora_local_ned_pos, 
                                             queue_size=0)
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

    def on_mavlink_msg(self, msg):
        if msg.msg_id == MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            self.publish_current_local_position(msg)
	
    def	publish_current_local_position(self, msg):
        #  'time_boot_ms', 'x', 'y', 'z', 'vx', 'vy', 'vz' ]
        (time_boot_ms, x, y, z, vx, vy, vz) = struct.unpack('<Iffffff', msg.payload)
        self.local_pos_msg.time_boot_ms = time_boot_ms
        self.local_pos_msg.x = x
        self.local_pos_msg.y = y
        self.local_pos_msg.z = z
        self.local_pos_msg.vx = vx
        self.local_pos_msg.vy = vy
        self.local_pos_msg.vz = vz
        self.local_ned_pub.publish(self.local_pos_msg)

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            # sleep the defined interval
            self.rate.sleep()

if __name__ == '__main__':
    pos = LocalPositionPublisher()
    pos.run()
	

