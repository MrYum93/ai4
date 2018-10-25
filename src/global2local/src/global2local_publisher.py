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
import pymap3d as pm

# topics
global2local_target_sub = "/set_position_target_global_geodetic"
global2local_target_pub = "/set_position_target_local_ned"
mavlink_lora_pos_sub_topic = '/mavlink_pos'

# messages 
from mavlink_lora.msg import mavlink_lora_local_setpoint, mavlink_lora_pos

# rate for sending setpoints - must be at least 2 hz
update_interval = 1

class Global2LocalCoordinate():
    def __init__(self):
        # launch node 
        rospy.init_node('global2local_node', disable_signals = True)
        self.setpoint_msg = mavlink_lora_local_setpoint()

        rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)

        rospy.Subscriber(global2local_target_sub, 
                         mavlink_lora_pos, 
                         self.on_global_geodetic_setpoint_msg)

        self.local_setpoint_pub = rospy.Publisher(global2local_target_pub, 
                                                  mavlink_lora_local_setpoint, 
                                                  queue_size=0)
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

        #variables
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_alt = 0.0
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.alt0 = 0.0
        # maybe we need relative alt also....

        rospy.sleep (2) # wait a little extra to make sure we got first callback
        self.set_initial_position()

    def set_initial_position(self):
        self.lat0 = self.lat
        self.lon0 = self.lon
        self.alt0 = self.alt
        print "init position lat: %02.7f lon: %02.7f, alt: %02.7f" % (self.lat0,
                                                                      self.lon0,
                                                                      self.alt0)

    def on_global_geodetic_setpoint_msg (self, msg):
        self.target_lat = msg.lat
        self.target_lon = msg.lon
        self.target_alt = msg.alt

        # convert to local target 
        north, east, up = pm.geodetic2ned(self.target_lat, 
                                 self.target_lon, 
                                 self.target_alt, 
                                 self.lat0, 
                                 self.lon0, 
                                 self.alt
                                 )
        # publish new target 
        self.setpoint_msg.x = north
        self.setpoint_msg.y = east
        self.setpoint_msg.z = -self.target_alt
        self.local_setpoint_pub.publish(self.setpoint_msg)

    def on_mavlink_lora_pos(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

    # def convert_geodetic_2_local_ned(self):
    #     # result = pm.geodetic2ned(self.target_lat, self.target_lon, self.target_alt, self.lat0, self.lon0, self.alt)
    #     # print (self.target_lat, self.target_lon, self.target_alt)
    #     # print result 

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            # sleep the defined interval
            self.rate.sleep()

if __name__ == '__main__':
    g2l = Global2LocalCoordinate()
    g2l.run()