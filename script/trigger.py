#!/usr/bin/env python
# from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import time
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math 


class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("/vrep/visual_trigger",String, queue_size=1)
		

def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	
	try:
		ic.image_pub.publish("on")
		time.sleep(5)
		ic.image_pub.publish("off")

		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)