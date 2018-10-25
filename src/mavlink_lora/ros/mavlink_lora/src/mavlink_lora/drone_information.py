#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
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
Adopted version of the gcs_node class. Used to collect drone information
'''

#msg ids
MAVLINK_MSG_ID_HEARTBEAT = 0

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
mavlink_lora_local_ned_pos_sub = '/mavlink_local_position_ned'
update_interval = 10

# imports
import rospy
import struct
from std_msgs.msg import Int8
from mavlink_lora.msg import (mavlink_lora_msg, mavlink_lora_status, 
							  mavlink_lora_pos, mavlink_lora_attitude,
							  mavlink_lora_local_ned_pos)
from math import pi, sqrt, sin, cos, atan2

class DroneInformation:
	def __init__(self):
		self.msg = mavlink_lora_msg()
		self.request_sent = False

		# status variables
		self.batt_volt = 0.0
		self.last_heard = 0
		self.last_heard_sys_status = 0
		self.lat = 0.0
		self.lon = 0.0
		self.alt = 0.0
		self.home_lat = 0.0
		self.home_lon = 0.0
		self.yaw = 0.0
		self.pitch = 0.0
		self.roll = 0.0
		self.local_x = 0.0
		self.local_y = 0.0
		self.local_z = 0.0
		self.local_vx = 0.0
		self.local_vy = 0.0
		self.local_vz = 0.0

		# launch node 
		# rospy.init_node('mavlink_lora_gcs_simple', disable_signals = True)
		# self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, 
						 self.on_mavlink_msg)
		rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, 
						 self.on_mavlink_lora_status)
		rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos,
						 self.on_mavlink_lora_pos)
		rospy.Subscriber(mavlink_lora_atti_sub_topic, mavlink_lora_attitude, 
						 self.on_mavlink_lora_attitude)
		rospy.Subscriber(mavlink_lora_local_ned_pos_sub, 
						 mavlink_lora_local_ned_pos, 
						 self.on_mavlink_lora_local_pos)

		self.rate = rospy.Rate(update_interval)
		rospy.sleep (1) # wait until everything is running

	def on_mavlink_lora_local_pos(self, msg):
		self.local_x = msg.x
		self.local_y = msg.y
		self.local_z = msg.z
		self.local_vx = msg.vx
		self.local_vy = msg.vy
		self.local_vz = msg.vz

	def on_mavlink_msg (self, msg):
		pass

	def on_mavlink_lora_status (self, msg):
		self.last_heard = msg.last_heard.secs + msg.last_heard.nsecs/1.0e9
		self.last_heard_sys_status = (msg.last_heard_sys_status.secs + 
									  msg.last_heard_sys_status.nsecs/1.0e9)
		self.batt_volt = msg.batt_volt / 1000.0
		
	def on_mavlink_lora_pos (self, msg):
		self.lat = msg.lat
		self.lon = msg.lon
		self.alt = msg.alt
		# print 'pos'

	def on_mavlink_lora_attitude (self, msg):
		self.yaw = msg.yaw
		self.pitch = msg.pitch
		self.roll = msg.roll

	def run(self):
		# loop until shutdown
		while not (rospy.is_shutdown()):
			# do stuff
			# sleep the defined interval
			self.rate.sleep()

if __name__ == '__main__':
	gcs = DroneInformation()
	gcs.run()
	

