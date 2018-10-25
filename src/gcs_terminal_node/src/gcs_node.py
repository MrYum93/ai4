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
This example script shows how to obtain basic status of a UAV.

NOT YET: It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-05-29 KJ First version
2018-10-08 Per Breum - add more terminal information
2018-10-09 Per Breum - add more keystrokes options
2018-10-12 Per Breum - add keystrokes to control drone micro movement
'''
# topics
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
mavlink_lora_keypress_sub_topic = '/keypress'
mavlink_lora_modes_pub_topic = '/mavlink_modes'
local_target_pub_topic = '/position_target_local_ned'
mavlink_lora_local_ned_pos_sub_topic = '/mavlink_local_position_ned'
mavlink_lora_set_local_target_pub_topic = "/set_position_target_local_ned"
drone_cmd_handler_arm_sub = "/drone_command_arm"
drone_cmd_handler_offboard_sub = "drone_command_offboard"
drone_cmd_handler_takeoff_sub = "drone_command_takeoff"
update_interval = 10

# imports
import rospy
import struct
from std_msgs.msg import Int8, Bool
from drone_command_handler.drone_cmd_handler import DroneCommandHandler
from mavlink_lora.msg import (mavlink_lora_msg, mavlink_lora_status, 
                             mavlink_lora_pos, mavlink_lora_attitude, 
                             mavlink_lora_modes, mavlink_lora_local_setpoint, mavlink_lora_local_ned_pos)
from math import pi, sqrt, sin, cos, atan2

# defines
R = 6371000 # Assumed Earth radius in meter
DEG2RAD = pi/180.0
RAD2DEG = 180.0/pi

# global
drone_dispacement = 100 # 100 cm

class gcs_node:
	def __init__(self):
		self.msg = mavlink_lora_msg()
		self.setpoint_msg = mavlink_lora_local_setpoint()
		self.request_sent = False
		self.key_text = 'Unknown'
	
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
		self.custom_main_mode = ''
		self.custom_sub_mode = ''
		self.local_alt_target = 0
		self.local_lat_target = 0
		self.local_lon_target = 0
		self.local_x = 0.0
		self.local_y = 0.0
		self.local_z = 0.0
		self.local_vx = 0.0
		self.local_vy = 0.0
		self.local_vz = 0.0

		# launch node
		rospy.init_node('gcs_node', disable_signals = True)
		self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, 
											   mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, 
						 self.on_mavlink_msg)
		rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, 
						 self.on_mavlink_lora_status)
		rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, 
						 self.on_mavlink_lora_pos)
		rospy.Subscriber(mavlink_lora_atti_sub_topic, mavlink_lora_attitude, 
						 self.on_mavlink_lora_attitude)
		rospy.Subscriber(mavlink_lora_keypress_sub_topic, Int8, 
						 self.on_keypress)
		rospy.Subscriber(mavlink_lora_modes_pub_topic, mavlink_lora_modes, 
						 self.modes_sub)
		rospy.Subscriber(local_target_pub_topic, mavlink_lora_local_setpoint, 
						 self.local_target_sub)
		rospy.Subscriber(mavlink_lora_local_ned_pos_sub_topic, 
						 mavlink_lora_local_ned_pos, 
						 self.on_mavlink_lora_local_pos)
		self.modes_pub = rospy.Publisher(mavlink_lora_set_local_target_pub_topic, 
                                    	  mavlink_lora_local_setpoint, 
                                    	  queue_size=0)
		self.arm_pub = rospy.Publisher(drone_cmd_handler_arm_sub, Bool, 
									   queue_size=0)
		self.offboard_pub = rospy.Publisher(drone_cmd_handler_offboard_sub, Bool, 
									   		queue_size=0)
		self.takeoff_pub = rospy.Publisher(drone_cmd_handler_takeoff_sub, Bool, 
									   	   queue_size=0)
		self.rate = rospy.Rate(update_interval)
		rospy.sleep (1) # wait until everything is running
		self.cmd = DroneCommandHandler()

	def on_mavlink_lora_local_pos(self, msg):
		self.local_x = msg.x
		self.local_y = msg.y
		self.local_z = msg.z
		self.local_vx = msg.vx
		self.local_vy = msg.vy
		self.local_vz = msg.vz

	def local_target_sub(self, msg):
		self.local_lat_target = msg.x
		self.local_lon_target = msg.y
		self.local_alt_target = msg.z

	def modes_sub(self, msg):
		self.custom_main_mode = msg.custom_main_mode_str
		self.custom_sub_mode = msg.custom_sub_mode_auto_str

	def gcd_haversine (self, lat1, lon1, lat2, lon2):
		lat1 *= DEG2RAD	
		lon1 *= DEG2RAD	
		lat2 *= DEG2RAD	
		lon2 *= DEG2RAD	
		dlat = lat2-lat1
		dlon = lon2-lon1
		a = sin(dlat/2.)**2 + sin(dlon/2.)**2 * cos(lat1) * cos(lat2)
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		return (R * c)

	def update_display (self):
		now = rospy.get_time()
		print '\033[2J' # clear screen
		print '', # go home

		# update last_heard
		t = now - self.last_heard
		if t < 86400:
			last_heard_text = '%ds' % t
		else:
			last_heard_text = 'Never'

		# update last status
		t_sys_status = now - self.last_heard_sys_status
		if t_sys_status < 86400:
			last_heard_status_text = '%ds' % t_sys_status
		else:
			last_heard_status_text = 'Never'

		# update battery status
		if self.batt_volt == 0 or t_sys_status > 60:
			batt_text = 'Unknown'
		else:
			batt_text = '%.1fV' % self.batt_volt

		# update global pos status
		if self.lat == 0 and self.lon == 0:
			pos_text = 'Unknown'
		else:
			pos_text = '%02.7f %03.7f' % (self.lat, self.lon)

		# update altitude status
		if self.alt == 0:
			alt_text = 'Unknown'
		else:
			alt_text = '%.1fm ' % (self.alt)

		# update distance status
		if self.home_lat == 0 and self.home_lon == 0:
			if self.lat == 0 and self.lon == 0:
				home_text = 'Unknown'
			else:
				home_text = 'Press h to set home position'
		else:
			home_text = '%.1fm' % self.gcd_haversine(self.lat, self.lon, 
													 self.home_lat, 
													 self.home_lon)

		# update attitude text
		if self.yaw == 0 and self.pitch == 0 and self.roll == 0:
			atti_text = 'Unknown'
		else:
			atti_text = ('Yaw: %03.1f Pitch: %03.1f Roll: %03.1f' % 
			(self.yaw*180/pi, self.pitch*180/pi, self.roll*180/pi))

		# update mode state
		if not self.custom_main_mode and not self.custom_sub_mode:
			custom_main_mode_text = 'Unknown'
			custom_sub_mode_text = 'Unknown'
		else:
			custom_main_mode_text = self.custom_main_mode
			custom_sub_mode_text = self.custom_sub_mode
		
		# update local setpoints
		if (self.local_alt_target == 0 and self.local_lat_target == 0 and
			self.local_lon_target == 0):
			local_alt_target_text = 'Unknown'
			local_lat_target_text = 'Unknown'
			local_lon_target_text = 'Unknown'
		else: 						  
			local_alt_target_text = self.local_alt_target
			local_lat_target_text = self.local_lat_target
			local_lon_target_text = self.local_lon_target

		# update local pos status
		if self.local_x == 0 and self.local_y == 0 and self.local_z == 0:
			local_pos_text = 'Unknown'
			local_alt_text = 'Unknown'
		else:
			local_pos_text = '%02.7f %03.7f' % (self.local_x,
												self.local_y)
			local_alt_text = '%.1fm ' % (self.local_z)

		print '\033[1HLast heard:         %s' % last_heard_text
		print '\033[2HLast system status: %s' % last_heard_status_text
		print '\033[4HBattery:            %s' % batt_text
		print '\033[6HGlobal Position:           %s' % pos_text
		print '\033[7HGlobal Altitude:           %s' % alt_text
		print '\033[9HLocal Position:           %s' % local_pos_text
		print '\033[10HLocal Altitude:           %s' % local_alt_text  
		print '\033[12HDistance:           %s' % home_text
		print '\033[13HAttitude:           %s' % atti_text
		# modes
		print '\033[15HCustom Main mode:  %s' % custom_main_mode_text
		print '\033[16HCustom Sub mode:  %s' % custom_sub_mode_text
		# local target setpoint
		print '\033[18HLocal target lat:  %s' % local_lat_target_text
		print '\033[19HLocal target lon:  %s' % local_lon_target_text
		print '\033[20HLocal target alt:  %s' % local_alt_target_text
		print '\033[22HKeypress options:'
		print '\033[23Hh=set home pos, q=quit, l=land, o=offboard, t=takeoff at pos (alt=5m), e=arm'
		print '\033[24HUse w,a,s,d to move drone around with a displacement of 100 cm'
		print '\033[?25l' # hide cursor
	
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

	def on_mavlink_lora_attitude (self, msg):
		self.yaw = msg.yaw
		self.pitch = msg.pitch
		self.roll = msg.roll

	def on_keypress (self, msg):
		if msg.data == ord('h'):
			if self.lat != 0 or self.lon != 0:
				self.home_lat = self.lat
				self.home_lon = self.lon
		elif msg.data == ord('q'):
			rospy.signal_shutdown('User quit')
		# land drone at position
		elif msg.data == ord('l'):
			self.cmd.land_at_pos()
		# activate offboard
		elif msg.data == ord('o'):
			self.offboard_pub.publish(True)
		# arm drone
		elif msg.data == ord('e'):
			self.arm_pub.publish(True)		
		# takeoff current pos
		elif msg.data == ord('t'):
			self.takeoff_pub.publish(True)
		elif (msg.data == ord('d') or 
			  msg.data == ord('a') or 
			  msg.data == ord('w') or 
			  msg.data == ord('s')):
			if msg.data == ord('d'):	
				self.setpoint_msg.x = self.local_x
				self.setpoint_msg.y = self.local_y + drone_dispacement/100
				self.setpoint_msg.z = self.local_z
			elif msg.data == ord('a'):
				self.setpoint_msg.x = self.local_x
				self.setpoint_msg.y = self.local_y - drone_dispacement/100
				self.setpoint_msg.z = self.local_z
			elif msg.data == ord('w'):
				self.setpoint_msg.x = self.local_x + drone_dispacement/100
				self.setpoint_msg.y = self.local_y
				self.setpoint_msg.z = self.local_z
			elif msg.data == ord('s'):
				self.setpoint_msg.x = self.local_x - drone_dispacement/100
				self.setpoint_msg.y = self.local_y
				self.setpoint_msg.z = self.local_z				
        	self.modes_pub.publish(self.setpoint_msg)

	def run(self):
		# loop until shutdown
		while not (rospy.is_shutdown()):
			# do stuff
			self.update_display()
			if self.request_sent == False:
				print 'Requesting...'
				self.request_sent = True	

			# sleep the defined interval
			self.rate.sleep()

if __name__ == '__main__':
	gcs = gcs_node()
	gcs.run()
	

