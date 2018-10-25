
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
MAV_CMD_COMPONENT_ARM_DISARM = 400
ARM = 1
DISARM = 0

# imports
from sys import argv
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

class ArmDisarmDrone():
    def __init__(self,arm_request):
        pass
        self.msg = mavlink_lora_msg()
        # status variables
        self.arm_request = arm_request
        self.request_sent = False
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.stop = False

        # initiate node
        rospy.init_node("mavlink_command_node") # change to variable
        self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_arm_msg)
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running

    def on_mavlink_arm_msg (self, msg):
        if msg.msg_id == MAVLINK_MSG_ID_COMMAND_ACK:
            (command, result)= struct.unpack('<HB', msg.payload)
            if command == 400 and result == 0:
                if self.arm_request_sent:
                    print 'Armed'
                elif self.disarm_request_sent:
                    print 'Disarmed'
            self.stop = True

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
        self.mavlink_msg_pub.publish(self.msg)
        self.disarm_request_sent = True

    def run(self):
        while not (rospy.is_shutdown() or self.stop):
            # do stuff
            if self.request_sent == False:
                if self.arm_request == ARM:
                    self.arm()
                    self.request_sent = True
                elif self.arm_request == DISARM:
                    self.disarm()
                    self.request_sent = True
            # sleep the defined interval
            self.rate.sleep()

# for test purposes
if __name__ == '__main__':
    if len(argv) == 1:
        print 'Usage: arm_disarm_drone.py [1 or 0]\n1=ARM\n0=DISARM'
    else:
        arm_disarm = int(argv[1])
        if arm_disarm == ARM:
            print 'Arming'
        elif arm_disarm == DISARM:
            print 'Disarming'
        arm = ArmDisarmDrone(arm_disarm)
        arm.run()


