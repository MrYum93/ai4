#!/usr/bin/env python

import sys
import rospy
from drone_command_handler.srv import *

def update_setpoint(x, y, z):
    rospy.wait_for_service('local_setpoint_manual_service')
    try:
        setp = rospy.ServiceProxy('local_setpoint_manual_service', setpoint)
        ret = setp(x, y, z)
        return ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [lat lon alt]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
    else:
        print usage()
        sys.exit(1)

    update_setpoint(x, y, z)