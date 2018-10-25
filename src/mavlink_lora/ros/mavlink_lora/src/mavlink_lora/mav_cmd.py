
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
update_interval = 10
target_system = 1 # Pixhawk2 PX4
target_component = 0

# defines
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_COMMAND_LONG_LEN = 33
MAV_CMD_NAV_TAKEOFF_LOCAL = 24
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_MISSION_START = 300
MAV_CMD_COMPONENT_ARM_DISARM = 400
ARM = 1
DISARM = 0

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

class MavlinkCommands:
    def __init__(self):
        self.msg = mavlink_lora_msg()

        # status variables
        self.armed = False
        self.arm_request_sent = False
        self.disarm_request_sent = False

        self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

    def on_mavlink_msg (self, msg):
        if msg.msg_id == MAVLINK_MSG_ID_COMMAND_ACK:
            (command, result)= struct.unpack('<HB', msg.payload)
            if command == 400 and result == 0:
                if self.arm_request_sent:
                    self.armed = True
                    self.arm_request_sent = False
                elif self.disarm_request_sent:
                    self.armed = False
                    self.disarm_request_sent = False

    def takeoff(self, lat, lon, alt, yaw_angle):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 0, 0, 0, yaw_angle, lat, lon, alt, MAV_CMD_NAV_TAKEOFF , target_system, target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)

    def start_mission(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', 1, 0, 0, 0, 0, 0, 0, MAV_CMD_MISSION_START , target_system, target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)

    def arm(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', ARM, 0, 0, 0, 0, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM , target_system, target_component, 0)
        self.mavlink_msg_pub.publish(self.msg)
        self.arm_request_sent = True

    def disarm(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', DISARM, 0, 0, 0, 0, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM , target_system, target_component, 0)
        self.mavlink_msg_pub.publish(msg)
        self.disarm_request_sent = True


# for test purposes
if __name__ == '__main__':
        # launch node
    rospy.init_node('mavlink_lora_arm_drone')
    # mavlink_msg publisher
    mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg,
                    queue_size=0)
    # mavlink_msg subscriber
    rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg)
    rate = rospy.Rate(update_interval)
    rospy.sleep (1) # wait until everything is running
    # loop until shutdown
    while not (rospy.is_shutdown()):
        # do stuff
        if armed == False:
            print 'arming'
            arm()
            print 'armed'
            # armed = True
        # sleep the defined interval
        rate.sleep()
