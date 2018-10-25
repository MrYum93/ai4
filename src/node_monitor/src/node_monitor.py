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
This script listens to all active interesting nodes and gives a heartbeat,
or calls them if no heartbeat. Inspiration from Per Breum, and from
https://github.com/ros/ros_comm/blob/melodic-devel/test/test_rospy/nodes/listener_once.py

NOT YET: Tested on mavlinkLora nodes from RAMUASD18

Revision
Aaaa-md-dd
2018-11-10 MWM - First version

Notes:
look at http://wiki.ros.org/rosservice for heartbeat
'''
# parameters
update_interval = 100

# sumscribers to monitor
beat_path_planner =  '/rate/pathplanner'
beat_gcs =           '/rate/gcs'
beat_dch =           '/rate/drone_command_ahndler'
beat_g2l =           '/rate/g2l'
# beat_mavlink_lora =  '/rate/mavlink_lora'
beat_utm_handler =   '/rate/utm_handler'
beat_mavlink_lora =  '/gcs_heartbeat'

# imports
import rospy
import rostopic
import rosnode
import rosmsg
from std_msgs.msg import Int8, String
from math import pi, sqrt, sin, cos, atan2
from astropy.table import Table, Column
from mavlink_lora.msg import (mavlink_lora_msg, mavlink_lora_status,
                                    mavlink_lora_pos, mavlink_lora_attitude,
                                    mavlink_lora_modes, mavlink_lora_local_setpoint,
                                    mavlink_lora_local_ned_pos, ) # maybe not all neccecary
from gcs_terminal_node.msg import gcs_heartbeat
from rosgraph_msgs.msg import Clock
from coord_pub.msg import coordList
from gazebo_msgs.msg import LinkStates

class node_monitor:
    def __init__(self):
        # launch node
        rospy.init_node('node_monitor', disable_signals=True)

        self.alive = "Dead"
        self.sol = 0 # timer for start of loop
        self.time_reset = rospy.get_time()
        self.now_path = rospy.get_time()
        self.now_gcs = rospy.get_time()
        self.now_dch = rospy.get_time()
        self.now_g2l = rospy.get_time()
        self.now_mav = rospy.get_time()
        self.last_mav = rospy.get_time()
        self.now_utm = rospy.get_time()
        # self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic,
        #                                        mavlink_lora_msg, queue_size=0)
        rospy.Subscriber(beat_path_planner, Int8, self.pathplanner_callback)
        rospy.Subscriber(beat_gcs         , Int8, self.gcs_callback)
        rospy.Subscriber(beat_dch         , Int8, self.dch_callback)
        rospy.Subscriber(beat_g2l         , Int8, self.g2l_callback)
        rospy.Subscriber(beat_mavlink_lora, gcs_heartbeat, self.mavlink_lora_callback)
        rospy.Subscriber(beat_utm_handler , Int8, self.utm_callback)

        self.rate = rospy.Rate(update_interval)
        rospy.sleep(1)  # wait until everything is running

        return

    def pathplanner_callback(self, msg):
        # hz = 20 has to be updated every 0.05 sec
        self.now_path = rospy.get_time()
        target_update = 1 / msg.data
        print(self.now_path - self.sol)

    def gcs_callback(self, msg):
        self.now_gcs = rospy.get_time()
        target_update = 1 / msg

    def dch_callback(self, msg):
        self.now_dch = rospy.get_time()
        target_update = 1 / msg

    def g2l_callback(self, msg):
        self.now_g2l = rospy.get_time()
        target_update = 1 / msg

    def mavlink_lora_callback(self, msg):
        self.now_mav = rospy.get_time()
        # target_update = 1/float(msg.data)
        print('Rate: ', 'target: ', 'actually: ', self.now_mav - self.last_mav)
        self.last_mav = rospy.get_time()


    def utm_callback(self, msg):
        self.now_utm = rospy.get_time()
        target_update = 1/msg

    def status(self):
        # If 1 sec has passed update to make it dead, stays this way if the node is actually dead
        self.update_alive = rospy.get_time()
        if self.update_alive - self.time_reset > 1:
            print(self.update_alive - self.time_reset)
            self.mon_clk_sub = ['/clock', 'Dead']

    def get_avail_topics(self):
        self.avail_topic = rospy.get_published_topics()
        return self.avail_topic

    def show_table(self):
        t = Table(names=('Node', 'Topic', 'Alive'), dtype=('S50', 'S50', 'S5'))
        i = 0
        alive = False
        node = ""
        topic = ""
        # t.add_row(('', self.mon_clk_sub[0], self.mon_clk_sub[1]))
        stringr = str(self.mon_clk_sub)
        stringrr = stringr.split("'")
        if len(stringrr) >= 2:
            print('mon clk sub ', stringrr[1], stringrr[3])
        for pck in self.get_avail_topics():
            node = str(pck[1])
            topic = str(pck[0])
            info = rostopic.get_info_text(topic)

            if info != None:
                alive = True
            # print(info)
            # l = rosmsg.list_types('rospy', mode='.msg')
            # print(l)
            # var = os.system("rosmsg info " + node + "\n")

            # message = rospy.wait_for_message(topic)
            # print(message)
            t.add_row((node, topic, alive)) # Adds all the topics to the table for monitoring

            i += 1
        print(t)

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff

            # if self.update_alive - self.time_reset > 5: # if 5 sec has passed update the nodes aliveness
            #     self.time_reset = rospy.get_time()
            # self.show_table()
            # self.still_alive()
            #all_nodes = rosnode.rosnode_ping('/rosout', max_count=1, verbose=False)
            #print(all_nodes)

            # sleep the defined interval
            self.rate.sleep()


if __name__ == '__main__':
    print('**** Start of program ****\n')
    print("list of avail topics:")
    listener = node_monitor()
    # listener.get_avail_topics()
    # listener.listener()
    listener.run()


    print('\n**** End of program ****')
