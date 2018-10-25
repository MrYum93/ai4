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
Receives POSITION_TARGET_LOCAL_NED (MSG #85) using mavlink_lora 
and publish information in topic /position_taget_local_ned

'''
import rospy
import struct

# msg ids
MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = 85

# topics
local_target_pub_topic = '/position_target_local_ned'
mavlink_lora_sub_topic = '/mavlink_rx'

# messages 
from mavlink_lora.msg import mavlink_lora_local_setpoint, mavlink_lora_msg

# rate for publishing setpoints
update_interval = 1

class LocalNEDSetpointPublisher():
    def __init__(self):
        # self.mavlink_msg = mavlink_lora_msg()
        self.setpoint_msg = mavlink_lora_local_setpoint()

        # launch node 
        rospy.init_node('local_setpoint_publisher', disable_signals = True)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, 
                         self.on_mavlink_msg)
        self.local_setpoint_pub = rospy.Publisher(local_target_pub_topic,
                                         mavlink_lora_local_setpoint, 
                                         queue_size=0)

        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running
 
    def on_mavlink_msg (self, msg):
        if msg.msg_id == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            self.publish_local_setpoints(msg)
            
    def publish_local_setpoints(self, msg):
        (self.setpoint_msg.time_boot_ms, 
         self.setpoint_msg.x, 
         self.setpoint_msg.y, 
         self.setpoint_msg.z, 
         self.setpoint_msg.vx, 
         self.setpoint_msg.vy, 
         self.setpoint_msg.vz, 
         self.setpoint_msg.afx, 
         self.setpoint_msg.afy, 
         self.setpoint_msg.afz, 
         self.setpoint_msg.yaw, 
         self.setpoint_msg.yaw_rate, 
         self.setpoint_msg.type_mask, 
         self.setpoint_msg.coordinate_frame) =  struct.unpack('<IfffffffffffHB', 
                                                              msg.payload)
        self.local_setpoint_pub.publish(self.setpoint_msg)

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            # sleep the defined interval
            self.rate.sleep()

if __name__ == '__main__':
    local_setpoint_pub = LocalNEDSetpointPublisher()
    local_setpoint_pub.run()