#!/usr/bin/env python
#/***************************************************************************
# info
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
info needed

'''
import rospy
import struct
from drone_command_handler.drone_cmd_handler import DroneCommandHandler

# msg ids
MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = 85

# topics
mavlink_lora_set_local_target_pub_topic = "/set_position_target_local_ned"
# maybe this should subscribe to a topic saying its okay to send 

# messages 
from mavlink_lora.msg import mavlink_lora_local_setpoint

# rate for sending setpoints - must be at least 2 hz
update_interval = 10

class TransmitLocalSetpoint():
    def __init__(self):
        # launch node 
        rospy.init_node('local_setpoint_node', disable_signals = True)
        self.setpoint_msg = mavlink_lora_local_setpoint()
                
        rospy.Subscriber(mavlink_lora_set_local_target_pub_topic, 
                         mavlink_lora_local_setpoint, 
                         self.on_local_setpoint_msg)

        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

        self.cmd = DroneCommandHandler()
        #variables
        self.x = 2
        self.y = 3
        self.z = -5
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.afx = 0
        self.afy = 0
        self.afz = 0
        self.yaw = 0
        self.yaw_rate = 0
        self.type_mask = 0
        self.coordinate_frame = 0


    def on_local_setpoint_msg (self, msg):
        self.setpoint_msg.time_boot_ms = msg.time_boot_ms
        self.setpoint_msg.x = msg.x
        self.setpoint_msg.y = msg.y
        self.setpoint_msg.z = msg.z
        # self.setpoint_msg.vx, 
        # self.setpoint_msg.vy, 
        # self.setpoint_msg.vz, 
        # self.setpoint_msg.afx, 
        # self.setpoint_msg.afy, 
        # self.setpoint_msg.afz, 
        # self.setpoint_msg.yaw, 
        # self.setpoint_msg.yaw_rate, 
        # self.setpoint_msg.type_mask, 
        # self.setpoint_msg.coordinate_frame
            
    def send_local_target_to_drone(self):
        self.cmd.set_local_ned_target(self.setpoint_msg.x, 
                                      self.setpoint_msg.y, 
                                      self.setpoint_msg.z)

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            self.send_local_target_to_drone()
            # sleep the defined interval
            self.rate.sleep()

if __name__ == '__main__':
    ls = TransmitLocalSetpoint()
    ls.run()