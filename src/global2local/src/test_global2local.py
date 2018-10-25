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
#test coordinates

lat = 55.471824
lon =  10.417470
alt = 10
# topics
global2local_target_pub = "/set_position_target_global_geodetic"

# messages 
from mavlink_lora.msg import mavlink_lora_pos

if __name__ == '__main__':
    rospy.init_node('global2local_test_node', disable_signals = True)
    setpoint_msg = mavlink_lora_pos()
    local_setpoint_pub = rospy.Publisher(global2local_target_pub, 
                                         mavlink_lora_pos, 
                                         queue_size=0)
    
    rate = rospy.Rate(5)
    rospy.sleep (1) # wait until everything is running
    setpoint_msg.lat = lat
    setpoint_msg.lon = lon
    setpoint_msg.alt = alt
    rospy.sleep (2)
    local_setpoint_pub.publish(setpoint_msg)
    while not (rospy.is_shutdown()):
        # do stuff
        
        # sleep the defined interval
        rate.sleep()