#!/usr/bin/env python

from drone_command_handler.drone_cmd_handler import DroneCommandHandler
import rospy

def main():
    rospy.init_node('test')
    # first = True
    # rospy.init_node('drone_command_handler')
    # cmd = MavlinkCommands()
    rate = rospy.Rate(10)
    cmd = DroneCommandHandler()
    rospy.sleep(1)
    cmd.set_gps_origin()
    rospy.sleep(1)
    while not (rospy.is_shutdown()):
        dad = 1
        rate.sleep()

if __name__ == "__main__":
    main()