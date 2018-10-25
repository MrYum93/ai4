#!/usr/bin/env python
#/***************************************************************************
# Handles heartbeat messages from GroundControlStation
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
Sending heartbeat messages from the GroundControlStation, GCS

'''
update_interval = 4

from gcs_terminal_node.msg import gcs_heartbeat

#topics
gcs_node_heartbeat_pub = "/gcs_heartbeat"

# for HEARTBEAT ( #0 )
custom_mode = 6 # offboard
typ = 18 # type -MAV_TYPE_ONBOARD_CONTROLLER - Onboard companion controller
autopilot = 0 # MAV_AUTOPILOT_GENERIC - Generic autopilot, full support for ever
base_mode = 8 # MAV_MODE_FLAG_GUIDED_ENABLED
system_status = 5 # MAV_STATE_ACTIVE
mavlink_version = 2

# from drone_command_handler.drone_cmd_handler import DroneCommandHandler
import rospy

def main():
    rospy.init_node('heartbeat_node')
    rate = rospy.Rate(update_interval)
    # cmd = DroneCommandHandler()

    heartbeat_pub = rospy.Publisher(gcs_node_heartbeat_pub, 
                                    gcs_heartbeat, 
                                    queue_size=0)
    # setup heartbeat message
    msg = gcs_heartbeat()
    msg.custom_mode = custom_mode
    msg.type = typ
    msg.autopilot = autopilot
    msg.base_mode = base_mode
    msg.system_status = system_status
    msg.mavlink_version = mavlink_version

    rospy.sleep(1)
    while not (rospy.is_shutdown()):
        heartbeat_pub.publish(msg)
        # cmd.send_heartbeat(custom_mode, typ, autopilot, base_mode, 
        #                    system_status, mavlink_version)
        rate.sleep()

if __name__ == "__main__":
    main()
