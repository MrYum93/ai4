#!/usr/bin/env python

import rospy
from drone_command_handler.srv import *
from mavlink_lora.msg import mavlink_lora_local_setpoint

# topics
mavlink_lora_set_local_target_pub_topic = "/set_position_target_local_ned"

class ServiceClassLocalSetpoint():
    def __init__(self):
        rospy.init_node('local_setpoint_manual_service_server')
        s = rospy.Service('local_setpoint_manual_service', setpoint, self.handle)
        print "Ready to receive manual setpoint updates"

        self.rate = rospy.Rate(10)
        self.setpoint_msg = mavlink_lora_local_setpoint()
        #variables
        self.setpoint_msg.x = 8
        self.setpoint_msg.y = 7
        self.setpoint_msg.z = 0
        self.setpoint_msg.vx = 0
        self.setpoint_msg.vy = 0
        self.setpoint_msg.vz = 0
        self.setpoint_msg.afx = 0
        self.setpoint_msg.afy = 0
        self.setpoint_msg.afz = 0
        self.setpoint_msg.yaw = 0
        self.setpoint_msg.yaw_rate = 0
        self.setpoint_msg.type_mask = 0
        self.setpoint_msg.coordinate_frame = 0
        rospy.sleep(1)

        self.modes_pub = rospy.Publisher(mavlink_lora_set_local_target_pub_topic, 
                                    mavlink_lora_local_setpoint, 
                                    queue_size=0)

    # only change setpoint when new msg is received
    def handle(self, req):
        print "Received lat lon lat: %f %f %f" % (req.x, req.y, req.z)
        self.setpoint_msg.x = req.x
        self.setpoint_msg.y = req.y
        self.setpoint_msg.z = req.z
        self.modes_pub.publish(self.setpoint_msg)


    def run(self):
        # rospy.spin()
        while not (rospy.is_shutdown()):
            self.rate.sleep()

if __name__ == "__main__":
    srv = ServiceClassLocalSetpoint()
    srv.run()