# #!/usr/bin/env python

# from mavlink_lora.drone_information import DroneInformation
# from mav_cmd import MavlinkCommands
# from coord_pub.msg import coordList
# import rospy

# class Coordinate():
#     def __init__(self):
#         self.lat = 0
#         self.lon = 0
#         self.alt = 0

# class DroneCommandHandler():
#     def __init__(self):
#         self.drone = DroneInformation()
#         self.mav_cmds = MavlinkCommands()
#         self.initial_position = self.get_initial_position()
#         self.coordSub = rospy.Subscriber("/coords",coordList, self.callback , queue_size=1)
	
# 	# Added by Thor 26/9 - Subscriber/callback
#     def callback(self,ros_data): 		
# 		rospy.loginfo(ros_data)
		
# # End of Thor add-ons 26/9
#     def arm(self):
#         self.mav_cmds.arm()

#     def activate_offboard(self):
#         # should there be a check if setpoints is published??
#         # try to activate offboard 5 times
#         # todo: add a check of the current mode
#         for x in range(5):
#             self.arm()
#             self.set_mode()
#             rospy.sleep(0.2)
#         # should start a timer with callback to report if offboard
#         # was never started

#     def get_initial_position(self):
#         return self.get_drone_position()
        
#     def get_drone_position(self):
#         pos = Coordinate()
#         pos.lat = self.drone.lat
#         pos.lon = self.drone.lon
#         pos.alt = self.drone.alt 
#         return pos 

#     def send_heartbeat(self):
#         self.mav_cmds.send_heartbeat()

#     def drone_takeoff(self, alt=10):    
#         self.mav_cmds.takeoff(self.initial_position.lat, 
#                               self.initial_position.lon,
#                               self.initial_position.alt+alt,
#                               yaw_angle=0)
                              
#     def fly_to_wp(self, lat, lon, alt, yaw=0):
#         self.mav_cmds.fly_to_waypoint(lat, lon, self.initial_position.alt+alt, 0)

#     def land_at_pos(self):
#         self.mav_cmds.land_at_position(self.drone.lat,
#                                        self.drone.lon,
#                                        self.drone.alt,
#                                        0)
#     def set_mode(self):
#         # self.mav_cmds.set_mode()
#         self.mav_cmds.set_mav_mode()

#     def clear_mission(self):
#         self.mav_cmds.clear_mission()
    
#     # NOT fully implemented yet
#     def upload_mission(self):
#         self.mav_cmds.upload_mission()

#     # STILL NEED TO FIGURE OUT TO SET INTO OFFBOARD TO USE THIS
#     def set_local_ned_target(self, lat, lon, alt):
#         self.mav_cmds.set_position_target_local_ned(lat, lon, alt)
    
#     # STILL NEED TO FIGURE OUT TO SET INTO OFFBOARD TO USE THIS
#     def set_global_ned_target(self):
#         self.mav_cmds.set_position_target_global_ned()

#     # STILL NEED TO FIGURE OUT TO SET INTO OFFBOARD TO USE THIS
#     def set_manual_setpoint(self):
#         self.mav_cmds.set_manual_setpoint()

# def main():
#     first = True
#     rospy.init_node('drone_command_handler')
#     cmd = DroneCommandHandler()

#     rate = rospy.Rate(10)
#     rospy.sleep(1)
    
#     cmd.mav_cmds.arm()
#     # cmd.drone_takeoff() 

#     # rospy.sleep(15)
#     # cmd.fly_to_wp(55.472182, 10.416948, 10.0, 1)
#     #cmd.mav_cmds.start_mission()
#     # cmd.drone_takeoff()    
#     # loop until shutdown
#     while not (rospy.is_shutdown()):
#         # print drone.lat
#     # if first:
#         # print 'first'
#         # cmd.arm_cmd.arm()
#         # print cmd.initial_position.lat
#         # print cmd.initial_position.lon
#         # print cmd.drone.lat
#         # print cmd.drone.lon
#         # print cmd.drone.alt
#         rate.sleep()
#     # Ddps = DronePositionSubscriber()


# if __name__ == "__main__":
#     main()
