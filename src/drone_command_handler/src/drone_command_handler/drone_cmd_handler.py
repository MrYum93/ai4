#!/usr/bin/env python
#/***************************************************************************
# Drone command handler
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
API for drone commands

'''
from mavlink_lora.drone_information import DroneInformation
from drone_command_handler import *
from mav_cmd import MavlinkCommands
from coord_pub.msg import coordList
from gcs_terminal_node.msg import gcs_heartbeat
import rospy
import sys
from std_msgs.msg import Bool
from mavlink_lora.msg import mavlink_lora_local_setpoint

#topics 
mavlink_lora_set_local_target_pub_topic = "/set_position_target_local_ned"
drone_cmd_handler_arm_sub = "/drone_command_arm"
drone_cmd_handler_takeoff_sub = "drone_command_takeoff"
drone_cmd_handler_offboard_sub = "drone_command_offboard"
gcs_node_heartbeat_sub = "/gcs_heartbeat"


class Coordinate():
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.alt = 0

class DroneCommandHandler():
    def __init__(self):
        self.drone = DroneInformation()
        self.setpoint_msg = mavlink_lora_local_setpoint()
        self.mav_cmds = MavlinkCommands()
        self.setpoint_msg = mavlink_lora_local_setpoint()
        self.initial_position = self.get_initial_position()
        self.coordSub = rospy.Subscriber("/coords",coordList, self.callback , queue_size=1)
        self.arm_sub = rospy.Subscriber(drone_cmd_handler_arm_sub,
                                        Bool, self.on_arm_msg, queue_size=1)
        self.offboard_sub = rospy.Subscriber(drone_cmd_handler_offboard_sub,
                                             Bool, self.on_offboard_msg, 
                                             queue_size=1)
        self.takeoff_sub = rospy.Subscriber(drone_cmd_handler_takeoff_sub,
                                            Bool, self.on_takeoff_msg, 
                                            queue_size=1)
        self.gcs_heartbeat = rospy.Subscriber(gcs_node_heartbeat_sub,
                                              gcs_heartbeat, 
                                              self.on_heartbeat_msg, 
                                              queue_size=1)
        self.local_setpoint_pub = rospy.Publisher(mavlink_lora_set_local_target_pub_topic, 
                                         mavlink_lora_local_setpoint, 
                                         queue_size=0)
	
	# Added by Thor 26/9 - Subscriber/callback
    def callback(self,ros_data): 		
		rospy.loginfo(ros_data)
		
# End of Thor add-ons 26/9
    def on_arm_msg(self, msg):
        if msg.data == True:
            self.arm()

    def arm(self):
        self.mav_cmds.arm()

    def on_offboard_msg(self, msg):
        if msg.data == True:
            self.activate_offboard()

    def activate_offboard(self):
        # should there be a check if setpoints is published??
        # try to activate offboard 5 times
        # todo: add a check of the current mode
        for x in range(5):
            self.arm()
            self.set_mode()
            rospy.sleep(0.2)
        # should start a timer with callback to report if offboard
        # was never started

    def get_initial_position(self):
        return self.get_drone_position()
        
    def get_drone_position(self):
        pos = Coordinate()
        pos.lat = self.drone.lat
        pos.lon = self.drone.lon
        pos.alt = self.drone.alt 
        return pos 

    def on_heartbeat_msg(self, msg):
        self.send_heartbeat(msg.custom_mode, msg.type, msg.autopilot, msg.base_mode, 
                       msg.system_status, msg.mavlink_version)

    def send_heartbeat(self,custom_mode, typ, autopilot, base_mode, 
                       system_status, mavlink_version):
        self.mav_cmds.send_heartbeat(custom_mode, typ, autopilot, base_mode, 
                                     system_status, mavlink_version)

    def on_takeoff_msg(self, msg):
        # print 'DADAWDAWD'
        # if msg.data == True:
        self.drone_takeoff()

    def drone_takeoff(self, alt=-5): 
        self.setpoint_msg.x = self.drone.local_x
        self.setpoint_msg.y = self.drone.local_y
        self.setpoint_msg.z = -5
        self.local_setpoint_pub.publish(self.setpoint_msg)
        rospy.sleep(0.1)
        self.activate_offboard()

        # self.mav_cmds.takeoff(self.initial_position.lat, 
        #                       self.initial_position.lon,
        #                       self.initial_position.alt+alt,
        #                       yaw_angle=0)
                              
    def fly_to_wp(self, lat, lon, alt, yaw=0):
        self.mav_cmds.fly_to_waypoint(lat, lon, self.initial_position.alt+alt, 0)

    def land_at_pos(self):
        self.mav_cmds.land_at_position(self.drone.lat,
                                       self.drone.lon,
                                       self.drone.alt,
                                       0)
    def set_mode(self):
        # self.mav_cmds.set_mode()
        self.mav_cmds.set_mav_mode()

    def clear_mission(self):
        self.mav_cmds.clear_mission()
    
    # NOT fully implemented yet
    def upload_mission(self):
        self.mav_cmds.upload_mission()

    # STILL NEED TO FIGURE OUT TO SET INTO OFFBOARD TO USE THIS
    def set_local_ned_target(self, x, y, z):
        self.mav_cmds.set_position_target_local_ned(x, y, z)
    
    # STILL NEED TO FIGURE OUT TO SET INTO OFFBOARD TO USE THIS
    def set_global_ned_target(self):
        self.mav_cmds.set_position_target_global_ned()

    # STILL NEED TO FIGURE OUT TO SET INTO OFFBOARD TO USE THIS
    def set_manual_setpoint(self):
        self.mav_cmds.set_manual_setpoint()

    def set_gps_origin(self):
        self.mav_cmds.set_gps_origin()

def main():
    first = True
    rospy.init_node('drone_command_handler')
    cmd = DroneCommandHandler()

    rate = rospy.Rate(10)
    rospy.sleep(1)
    
    cmd.mav_cmds.arm()
    # cmd.drone_takeoff() 

    # rospy.sleep(15)
    # cmd.fly_to_wp(55.472182, 10.416948, 10.0, 1)
    #cmd.mav_cmds.start_mission()
    # cmd.drone_takeoff()    
    # loop until shutdown
    while not (rospy.is_shutdown()):
        # print drone.lat
    # if first:
        # print 'first'
        # cmd.arm_cmd.arm()
        # print cmd.initial_position.lat
        # print cmd.initial_position.lon
        # print cmd.drone.lat
        # print cmd.drone.lon
        # print cmd.drone.alt
        rate.sleep()
    # Ddps = DronePositionSubscriber()


if __name__ == "__main__":
    main()
