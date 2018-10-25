#!/usr/bin/env python
#/***************************************************************************
# Handler for Mavlink commands
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
API for Mavlink messages
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
update_interval = 10
target_system = 1 # Pixhawk2 PX4
target_component = 0

# defines
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_COMMAND_LONG_LEN = 33
MAV_CMD_NAV_TAKEOFF_LOCAL = 24
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_MISSION_START = 300
MAV_CMD_COMPONENT_ARM_DISARM = 400
ARM = 1
DISARM = 0

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21

MAV_MODE_FLAG_STABILIZE_ENABLED = 16
MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_COUNT_LEN = 4
MAVLINK_MSG_ID_MISSION_ITEM = 39
MAVLINK_MSG_ID_MISSION_ITEM_INT = 73
MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN = 37
MAVLINK_MSG_ID_MISSION_CLEAR_ALL = 45
MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN = 2
MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51

MAVLINK_MSG_ID_SET_MODE = 11
MAVLINK_MSG_ID_SET_MODE_LEN = 6


MAV_MODE_GUIDED_ARMED = 216
MAV_MODE_TEST_ARMED = 194
MAV_MODE_MANUAL_ARMED = 192
MAV_MODE_STABILIZE_ARMED = 208
MAV_MODE_AUTO_ARMED = 220

MAV_CMD_NAV_GUIDED_ENABLE = 92

MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN = 53

MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = 86
MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN = 53
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

MAVLINK_MSG_ID_MANUAL_SETPOINT = 81
MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN = 22

MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_HEARTBEAT_LEN = 9

MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = 85
MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN = 51

MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48
MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN = 13

# PX4 custom main modes

# PX_CUSTOM_MAIN_MODES = ["DUMMY_MAIN_MODE",
#                         "MANUAL", 
#                         "ALT_CTL", 
#                         "POS_CTL", 
#                         "AUTO", 
#                         "ACRO", 
#                         "OFFBOARD", 
#                         "STABILIZED", 
#                         "RATTITUDE", 
#                         "SIMPLE"]

# PX4_CUSTOM_SUB_MODE_AUTO = ["DUMMY_SUBMODE",
#                             "READY",
#                             "TAKEOFF",
#                             "LOITER",
#                             "MISSION",
#                             "RTL",
#                             "LAND",
#                             "RTGS",
#                             "FOLLOW_TARGET",
#                             "PRECLAND"]

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

class MavlinkCommands:
    def __init__(self):
        self.msg = mavlink_lora_msg()

        # status variables
        self.armed = False
        self.arm_request_sent = False
        self.disarm_request_sent = False

        # initiate node
        # rospy.init_node("mavlink_command_node") # change to variable
        self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, 
                                               mavlink_lora_msg, queue_size=0)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, 
                         self.on_mavlink_msg)
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

    def on_mavlink_msg (self, msg):
        # if msg.msg_id == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            # (time_boot_ms, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, type_mask, coordinate_frame) =  struct.unpack('<IfffffffffffHB', msg.payload)
            # print 'x: {}, y: {}, z: {}, frame {}, vx: {}, yaw_rate: {}'.format(x,y,z,coordinate_frame, vx, yaw_rate)
        # print msg
        # if msg.msg.id == MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        #     print 'MAVLINK_MSG_ID_MISSION_REQUEST_INT'
        if msg.msg_id == MAVLINK_MSG_ID_COMMAND_ACK:
            (command, result)= struct.unpack('<HB', msg.payload)
            if command == 400 and result == 0:
                if self.arm_request_sent:
                    self.armed = True
                    self.arm_request_sent = False
                elif self.disarm_request_sent:
                    self.armed = False
                    self.disarm_request_sent = False
    
    #  'latitude', 'longitude', 'altitude', 'target_system' ]
    def set_gps_origin(self):
        print 'gps_origin'
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN
        self.msg.payload_len = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_LEN
        self.msg.payload = struct.pack('<iiiB', 
                                        554720090, #'latitude'
                                        104156470, #'longitude'
                                        490, #'altitude'
                                        target_system #'target_system'
                                        )
        self.mavlink_msg_pub.publish(self.msg)

    # need to use updated data in some of the parameters
    def send_heartbeat(self, custom_mode, typ, autopilot, base_mode, 
                       system_status, mavlink_version):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_HEARTBEAT
        self.msg.payload_len = MAVLINK_MSG_ID_HEARTBEAT_LEN
        self.msg.payload = struct.pack('<IBBBBB', 
                                        custom_mode, #'custom_mode'
                                        typ, #'type'
                                        autopilot, #'autopilot'
                                        base_mode, #'base_mode'
                                        system_status, #'system_status'
                                        mavlink_version #'mavlink_version'
                                        )
        # print 'heartbeat'
        self.mavlink_msg_pub.publish(self.msg)
    
    def get_local_position_target_ned(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
        self.msg.payload_len = MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN
        self.msg.payload = struct.pack('<IfffffffffffHB', 
                                        6, #'custom_mode'
                                        18, #'type'
                                        0, #'autopilot'
                                        8, #'base_mode'
                                        5, #'system_status'
                                        2 #'mavlink_version'
                                        )
        # print 'heartbeat'
        self.mavlink_msg_pub.publish(self.msg)

    # not used
    def set_manual_setpoint(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_MANUAL_SETPOINT
        self.msg.payload_len = MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN
        # ordered_fieldnames = [ 'time_boot_ms', 'roll', 'pitch', 'yaw', 'thrust', 
        # 'mode_switch', 'manual_override_switch' ]
        self.msg.payload = struct.pack('<IffffBB', 
                                        0, #'time_boot_ms', 
                                        1, # 'roll', 
                                        1, # 'pitch', 
                                        1, # 'yaw', 
                                        1, # 'thrust', 
                                        1, # 'mode_switch',
                                        1 #  'manual_override_switch' ]
                                        )
        # print 'manual setpoin'
        self.mavlink_msg_pub.publish(self.msg)

    # still hardcoded values atm - not working
    def set_position_target_global_ned(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
        self.msg.payload_len = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN
        #      ordered_fieldnames = [ 'time_boot_ms', 'lat_int', 'lon_int', 'alt', 'vx', 
        # 'vy', 'vz', 'afx', 'afy', 'afz', 'yaw', 'yaw_rate', 'type_mask', 
        # 'target_system', 'target_component', 'coordinate_frame' ]
        self.msg.payload = struct.pack('<IiifffffffffHBBB', 
                                        0, # timestamp
                                        554720090, # lat
                                        104156470, # lon
                                        -10.0, # alt
                                        1.0, # vx
                                        1.0, # vy
                                        1.0, # vz
                                        1.0, # afx
                                        1.0, # afy
                                        1.0, # afz
                                        1.0, # yaw
                                        1.0, # yaw_rate
                                        0b0000111111111000, # mask specifying use-only-lat-lon-alt
                                        target_system, # target system_id
                                        target_component, # target component id
                                        5
                                        )
        # print 'set global target ned'
        self.mavlink_msg_pub.publish(self.msg)

    # still hardcoded values atm
    def set_position_target_local_ned(self, lat, lon, alt):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED
        self.msg.payload_len = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN
        self.msg.payload = struct.pack('<IfffffffffffHBBB', 
                                        0, #'time_boot_ms' 
                                        lat, #'x'
                                        lon, #'y'
                                        alt, # 'z'
                                        1, # 'vx'
                                        1, # 'vy'
                                        1, # 'vz'
                                        1, # 'afx'
                                        1, # 'afy'
                                        1, #  'afz'
                                        0, #  'yaw'
                                        1, #  'yaw_rate'
                                        0b0000111111111000, #  'type_mask' # 0b0000111111111000
                                        target_system, #  'target_system'
                                        target_component, #  'target_component'
                                        7 #  'coordinate_frame'
                                        )
        # print 'mode mav mode'
        self.mavlink_msg_pub.publish(self.msg)

    def set_mav_mode(self, mode=1):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 
                                        True, #param1 
                                        0, #param2
                                        0, #param3
                                        0, #param4 
                                        0, #param5
                                        0, #param6
                                        0, #param7
                                        MAV_CMD_NAV_GUIDED_ENABLE, #command
                                        target_system, 
                                        target_component, 
                                        1 #confirmation
                                        )
        # print 'mode mav mode'
        self.mavlink_msg_pub.publish(self.msg)

    def set_mode(self):
        # ordered_fieldnames = [ 'custom_mode', 'target_system', 'base_mode' ]
        # format = '<IBB'
        # MAVLINK_MSG_ID_SET_MODE = 11
        # MAV_MODE_MANUAL_ARMED = 192

        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_SET_MODE
        self.msg.payload_len = MAVLINK_MSG_ID_SET_MODE_LEN
        self.msg.payload = struct.pack('<IBB', 4, 
                                        target_system, 
                                        MAV_MODE_STABILIZE_ARMED)
        self.mavlink_msg_pub.publish(self.msg)
        # print 'new mode'

    def upload_mission(self, mission=False):
        self.upload_mission_send_count(1)
    
    def upload_mission_send_count(self, count=1):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_MISSION_COUNT
        self.msg.payload_len = MAVLINK_MSG_ID_MISSION_COUNT_LEN
        self.msg.payload = struct.pack('<HBB', 10, 
                                        target_system, 
                                        target_component)
        self.mavlink_msg_pub.publish(self.msg)
        print 'upload count sent'

    def clear_mission(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_MISSION_CLEAR_ALL
        self.msg.payload_len = MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN
        self.msg.payload = struct.pack('<BB', 
                                        target_system, 
                                        target_component)
        self.mavlink_msg_pub.publish(self.msg)

    def takeoff(self, lat, lon, alt, yaw_angle):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 0, 0, 0, yaw_angle, lat, lon, 
                                       alt, MAV_CMD_NAV_TAKEOFF , target_system, 
                                       target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)

    def start_mission(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 1, 0, 0, 0, 0, 0, 0, 
                                       MAV_CMD_MISSION_START, 
                                       target_system, target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)

    def arm(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', ARM, 0, 0, 0, 0, 0, 0, 
                                       MAV_CMD_COMPONENT_ARM_DISARM, 
                                       target_system, target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)
        self.arm_request_sent = True

    def disarm(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', DISARM, 0, 0, 0, 0, 0, 0, 
                                       MAV_CMD_COMPONENT_ARM_DISARM, 
                                       target_system, target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)
        self.disarm_request_sent = True

    # not working atm.
    def fly_to_waypoint(self, lat, lon, alt, yaw_angle, hold_time=10000.0, 
                        accept_rad=2.0):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 
                                        hold_time, #param1
                                        accept_rad, #param2
                                        0, #param3
                                        yaw_angle, #param4 
                                        lat, #param5
                                        lon, #param6
                                        alt, #param7
                                        MAV_CMD_NAV_WAYPOINT, #command
                                        target_system, 
                                        target_component, 
                                        0 #confirmation
                                        )
        self.mavlink_msg_pub.publish(self.msg)

    def land_at_position(self, lat, lon, alt, yaw_angle):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 
                                        0, #param1
                                        0, #param2
                                        0, #param3
                                        yaw_angle, #param4 
                                        lat, #param5
                                        lon, #param6
                                        alt, #param7
                                        MAV_CMD_NAV_LAND, #command
                                        target_system, 
                                        target_component, 
                                        0 #confirmation
                                        )
        print 'land'
        self.mavlink_msg_pub.publish(self.msg)




# for test purposes
if __name__ == '__main__':
        # launch node
    rospy.init_node('mavlink_lora_arm_drone')
    # mavlink_msg publisher
    mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg,
                    queue_size=0)
    # mavlink_msg subscriber
    rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg)
    rate = rospy.Rate(update_interval)
    rospy.sleep (1) # wait until everything is running
    # loop until shutdown
    while not (rospy.is_shutdown()):
        # do stuff
        if armed == False:
            print 'arming'
            arm()
            print 'armed'
            # armed = True
        # sleep the defined interval
        rate.sleep()
