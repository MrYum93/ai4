#!/usr/bin/env python
#/***************************************************************************
# Publish drone modes received from PX4 heartbeat message
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
Receives heartbeat message from mavlink_rx topic and publish current 
drone modes to mavlink_modes topic
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_modes_pub_topic = '/mavlink_modes'
delete_again = '/rate/mavlink_lora'		#DELETE AGAIN
update_interval = 10

# msg ids
MAVLINK_MSG_ID_HEARTBEAT = 0

#px4 modes
PX_CUSTOM_MAIN_MODES = ["DUMMY_MAIN_MODE",
                        "MANUAL", 
                        "ALT_CTL", 
                        "POS_CTL", 
                        "AUTO", 
                        "ACRO", 
                        "OFFBOARD", 
                        "STABILIZED", 
                        "RATTITUDE", 
                        "SIMPLE"]

PX4_CUSTOM_SUB_MODE_AUTO = ["NOT USED",
                            "READY",
                            "TAKEOFF",
                            "LOITER",
                            "MISSION",
                            "RTL",
                            "LAND",
                            "RTGS",
                            "FOLLOW_TARGET",
                            "PRECLAND"]

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_modes
from std_msgs.msg import Int8			#DELETE AGAIN

class DroneModePublisher:
    def __init__(self):
        # self.msg = mavlink_lora_msg() # might be removed
        self.msg_mode = mavlink_lora_modes()
	self.data_send = Int8()			#DELETE AGAIN

        # launch node 
        rospy.init_node('drone_mode_publisher', disable_signals = True)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, 
                         self.on_mavlink_msg)
        self.modes_pub = rospy.Publisher(mavlink_lora_modes_pub_topic, 
                                         mavlink_lora_modes, 
                                         queue_size=0)
	self.delete_pub = rospy.Publisher(delete_again, Int8, queue_size=0)		#DELETE AGAIN
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

    def on_mavlink_msg (self, msg):
        if msg.msg_id == MAVLINK_MSG_ID_HEARTBEAT:
            self.publish_current_drone_modes(msg)
	
    def	publish_current_drone_modes(self, msg):
        (custom_mode,typ,autopilot,base_mode,system_status,mavlink_version) = (
            struct.unpack('<IBBBBB', msg.payload))
        submode = custom_mode >> 24 
        main_mode = (custom_mode >> 16) & 0xFF

        self.msg_mode.custom_main_mode = main_mode
        self.msg_mode.custom_main_mode_str = PX_CUSTOM_MAIN_MODES[main_mode]
        self.msg_mode.custom_sub_mode_auto = submode
        self.msg_mode.custom_sub_mode_auto_str = (
            PX4_CUSTOM_SUB_MODE_AUTO[submode])
        self.modes_pub.publish(self.msg_mode)
	self.data_send.data = update_interval			#DELETE AGAIN
	self.delete_pub.publish(self.data_send)			#DELETE AGAIN

        # self.print_modes_to_console(main_mode, submode)

    def print_modes_to_console(self, main_mode, submode):
        print 'MAIN_MODE: {}, \
               Dec {}, \
               MODE: {:s}'.format(bin(main_mode),
                                  main_mode, 
                                  PX_CUSTOM_MAIN_MODES[main_mode])
        print 'SUBMODE: {}, \
               Dec {}, \
               MODE: {:s}'.format(bin(submode),
                                  submode,
                                  PX4_CUSTOM_SUB_MODE_AUTO[submode])
    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            
            # sleep the defined interval
            self.rate.sleep()

if __name__ == '__main__':
    dmp = DroneModePublisher()
    dmp.run()
	

