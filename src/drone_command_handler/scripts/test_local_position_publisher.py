#!/usr/bin/env python

# from drone_command_handler.drone_cmd_handler import DroneCommandHandler
from mavlink_lora.msg import mavlink_lora_local_setpoint
import rospy

# topics
mavlink_lora_set_local_target_pub_topic = "/set_position_target_local_ned"

def main():
    rospy.init_node('test_pos_publisher')
    rate = rospy.Rate(10)
    # cmd = DroneCommandHandler()
    setpoint_msg = mavlink_lora_local_setpoint()
    #variables
    setpoint_msg.x = 8
    setpoint_msg.y = 7
    setpoint_msg.z = 0
    setpoint_msg.vx = 0
    setpoint_msg.vy = 0
    setpoint_msg.vz = 0
    setpoint_msg.afx = 0
    setpoint_msg.afy = 0
    setpoint_msg.afz = 0
    setpoint_msg.yaw = 0
    setpoint_msg.yaw_rate = 0
    setpoint_msg.type_mask = 0
    setpoint_msg.coordinate_frame = 1
    rospy.sleep(1)

    modes_pub = rospy.Publisher(mavlink_lora_set_local_target_pub_topic, 
                                mavlink_lora_local_setpoint, 
                                queue_size=0)

    while not (rospy.is_shutdown()):
        modes_pub.publish(setpoint_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
