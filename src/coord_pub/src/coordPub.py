#! /usr/bin/env python 

import rospy
import sys
import signal
from coord_pub.msg import coordList



def main(array):
	myArray = []
	coordPub = rospy.Publisher("/coords", coordList,queue_size=1)
	rospy.loginfo("hello ROS")
	rospy.loginfo(array)
	myArray = coordList()
	myArray.data = array	
	while not rospy.is_shutdown():
		rate = rospy.Rate(1)	
		coordPub.publish(myArray)
		rate.sleep()	

	

if __name__ == '__main__':
	rospy.init_node('Coordinate_publisher',anonymous=False)
	myArray =[1.1,1231,2.2]
	try:
		main(myArray)
		rospy.spin()
	except KeyboardInterrupt:
		print("fuuuck")
		
