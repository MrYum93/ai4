#!/usr/bin/env python

from drone_cmd_handler import DroneCommandHandler
import rospy
def my_callback(event):
    print 'Timer called at ' + str(event.current_real)
    cmd = DroneCommandHandler()
    cmd.upload_mission()

# TEST inferface, just ignore for now
def main():
    rospy.init_node('test')
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
    
    # cmd.arm()
    
    # cmd.fly_to_wp(55.472182, 10.416948, 10.0, 1)
    # cmd.land_at_pos()
    # #cmd.mav_cmds.start_mission()
    # cmd.drone_takeoff()    
    # # loop until shutdown
    while not (rospy.is_shutdown()):
        cmd.set_local_ned_target()
        # cmd.set_global_ned_target()
        rate.sleep()

if __name__ == "__main__":
    main()
