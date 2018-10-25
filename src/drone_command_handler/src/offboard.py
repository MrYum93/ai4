#!/usr/bin/env python

from drone_cmd_handler import DroneCommandHandler
import rospy


def main():
    rospy.init_node('offboard')
    # first = True
    # rospy.init_node('drone_command_handler')
    # cmd = MavlinkCommands()
    rate = rospy.Rate(10)
    cmd = DroneCommandHandler()

    rospy.sleep(1)

    
    # cmd.set_global_ned_target()
    # for x in range(120):
    #     cmd.arm()
    #     cmd.set_manual_setpoint()
    
    # # rospy.Timer(rospy.Duration(0.25), my_callback)

    # rospy.sleep(1)
    cmd.activate_offboard()
    # cmd.arm()

    # for x in range(15):
    #     cmd.set_mode()
    #     rospy.sleep(0.2)
 
    while not (rospy.is_shutdown()):
        
        rate.sleep()

if __name__ == "__main__":
    main()
