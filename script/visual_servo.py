#!/usr/bin/env python
# from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math 

global x_i_sum
global servo_on
x_i_sum = 0.
servo_on = 0
last_angular_z = 0.

car_vel_pub = rospy.Publisher("/key_vel", Twist, queue_size=1)

def mask_center(mask):
	(rows, cols) = mask.shape

	ball_pixel_sum = 0.0

	for i in range(rows):
		for j in range(cols):
			if (mask[i, j] == 255):
				ball_pixel_sum = ball_pixel_sum + 1.0

	center_i = 0.0
	center_j = 0.0

	for i in range(rows):
		for j in range(cols):
			if (mask[i, j] == 255):
				center_i = center_i + i * 1.0 / ball_pixel_sum
				center_j = center_j + j * 1.0 / ball_pixel_sum
	
	#compute r	
	pi = 3.1415926
	r = math.sqrt(float(ball_pixel_sum)/pi)
	print "r =", r

	#return (x,y) of center

	return [center_j, center_i, r]

def compute_vel(x, r):
	global x_i_sum
	global last_angular_z

	target_r = 130
	target_x = 0
	dx = -x	
	dr = target_r - r

	x_i_sum += dx
	

	r_kp = 0.5
	r_ki = 0
	r_kd = 0
	r_p = r_kp * dr
	r_i = r_ki * 1
	
	x_kp = 1	
	x_ki = 0.0
	x_kd = 0
	x_p = x_kp * dx
	x_i = x_ki * x_i_sum
	

	car_vel = Twist()
	if (r > 40):
		if (x_p + x_i  > 0.08):
			car_vel.angular.z = 0.5
		elif (x_p + x_i < -0.08):
			car_vel.angular.z = -0.5
		else :
			car_vel.angular.z = 0
		last_angular_z = car_vel.angular.z
	else:
		car_vel.angular.z = last_angular_z * 1.2

	car_vel.linear.x = r_p + r_i 
	if (r_p + r_i  > 5):
		car_vel.linear.x = 0.4
	elif (r_p + r_i  < -8):
		car_vel.linear.x = -0.15
	else:
		car_vel.linear.x = 0

	if (r < 40):
		car_vel.linear.x = 0


	car_vel_pub.publish(car_vel)
	print 'dx = ', dx
	print 'dr = ', dr,', r =' r 
	print 'vel = ', car_vel



class image_converter:

	def __init__(self):
		# self.image_pub = rospy.Publisher("image_topic_2",Image)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/vrep/image",Image,self.callback,queue_size=1)
		self.trigger_sub = rospy.Subscriber("/vrep/visual_trigger",String,self.trigger_callback,queue_size=1)


	def callback(self,data):
		global servo_on
		try:
			cv_image_filpped = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv2.flip(cv_image_filpped, 1)
			# cv_image = imread()
		except CvBridgeError as e:
			print(e)

		(rows,cols,channels) = cv_image.shape
		# if cols > 60 and rows > 60 :
		# 	cv2.circle(cv_image, (50,50), 10, 255)

		img_HSV = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
		
		mask = cv2.inRange(img_HSV, np.array([10,180,40]), np.array([45,255,255]))
		
		(center_x, center_y, ball_r) = mask_center(mask)
		# print [int(center[0]),int(center[1])]
		if (center_x != 0):
			cv2.circle(cv_image, (int(center_x),int(center_y)), int(ball_r), 255)
		
		if (servo_on == 1):
			compute_vel(float(center_x - cols/2) /cols , ball_r)

	def trigger_callback(self, data):
		global servo_on
		if (data.data == 'on'):
			servo_on = 1
			print 'visual servo on'
		elif (data.data == 'off'):
			servo_on = 0
			print 'visual servo off'
		else:
			print 'unknown command'

def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)