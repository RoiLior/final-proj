#!/usr/bin/env python

import rospy
import os
import cv2
import numpy as np
import binascii
import struct
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import time

dist = -1
pc2_msg = -1
min_dist = -1
ranges = -1
door_ranges = -1
sight_ranges = -1

# Call-back for getting massage from 'scan' sensor and update relevant params
def update_ranges(msg):
	global ranges,dist,min_dist,door_ranges,sight_ranges

	ranges = [-1 if (math.isinf(i) or math.isnan(i)) else i for i in msg.ranges]
	door_ranges = ranges[100:412]
	sight_ranges = msg.ranges[246:257]

	dist = msg.ranges[len(msg.ranges)/2]
	min_dist = min(msg.ranges[150:362])


# Call-back for getting massage from 'torso_camera' sensor and update relevant params
def update_image(msg):
	global pc2_msg

	pc2_msg = msg


# STEP1 : Moving forward from starting point 
def step1(twist_publisher):
	global dist
	move_until(twist_publisher,1.7)

# STEP2 : Turning towards the door
def step2(twist_publisher):
	turn(twist_publisher,-90)

# STEP3 : Advancing towards the door, until reaching the doors exit, aiming to the center of the door
def step3(twist_publisher):
	global door_ranges

	turn_msg = Twist()
	door_indices = [0]
	while (max(door_indices) - min(door_indices)) < 120:
		mid_door = 0;
		while (abs(mid_door - len(door_ranges)/2) > 10):
			try:
				door_dist = max(door_ranges)
				indices = range(0,len(door_ranges))
				z = zip(indices, door_ranges)
				door_indices = [i[0] for i in z if i[1] > (door_dist - 0.5)]
				if len(door_indices) > 30:
					mid_door = (max(door_indices) + min(door_indices))/2

					if mid_door != len(door_ranges)/2:
						d = (mid_door - len(door_ranges)/2)/abs(mid_door - len(door_ranges)/2)
						turn_msg.angular.z = d * 0.2
						twist_publisher.publish(turn_msg)

				else: # BUG
					print "wrong scan"

			except Exception: # BUG
				pass

		move_forward(twist_publisher, 0.15)

# STEP4 : Stepping out of the door
def step4(twist_publisher):
	global door_ranges, dist

	turn_msg = Twist()
	door_indices = [0]
	while (max(door_indices) - min(door_indices)) < 200:
		mid_wall_idx = 0
		while (abs(mid_wall_idx - len(door_ranges)/2) > 10):
			max_range = max(door_ranges)
			indices = range(0,len(door_ranges))
			z = zip(indices, door_ranges)
			door_indices = [i[0] for i in z if i[1] > (max_range - 1)]
			if len(door_indices) > 30:
				min_idx = min(door_indices)
				max_idx = max(door_indices)
				mid_wall_dist = min(door_ranges[min_idx:max_idx])
				if mid_wall_dist > 0:
					mid_wall_idx = (max_idx + min_idx)/2

					if mid_wall_idx != len(door_ranges)/2:
						d = (mid_wall_idx - len(door_ranges)/2)/abs(mid_wall_idx - len(door_ranges)/2)

						turn_msg.angular.z = d * 0.15
						twist_publisher.publish(turn_msg)
				else: # BUG
					print "0 value", mid_wall_dist

			else: # BUG
				print "wrong scan"

		move_forward(twist_publisher, 0.1)

# STEP5 : Advancing and turning towards the hallway
def step5(twist_publisher):
	move_until(twist_publisher,1)
	turn(twist_publisher,100)
	move_forward(twist_publisher, 0.3)

# STEP6 : Passing the hallway, while keeping away from the sides, until reaching the end of the hallway
def step6(twist_publisher):
	global sight_ranges, ranges, dist

	move_msg = Twist()
	while dist > 0.55 or math.isnan(dist):
		while ranges[len(ranges)-1] > 0.5 and ranges[len(ranges)-1] <= 1:
			move_msg.linear.x = 0.2
			twist_publisher.publish(move_msg)

			if dist <= 0.55:
				break

		if ranges[len(ranges)-1] > 1:
			d = -1
		else:
			d = 1

		if dist > 0.55:
			turn(twist_publisher,0.5*d)

# STEP7 : Turning towards the lobby
def step7(twist_publisher):
	global sight_ranges, ranges, dist

	while not math.isinf(sight_ranges[len(sight_ranges)-1]):
		turn(twist_publisher,0.15)

# STEP8 : Advancing towards the outer lobby , while keeping away from the couches
def step8(twist_publisher):
	global dist

	move_msg = Twist()
	start = time.time()
	while time.time() - start < 11 and (math.isnan(dist) or dist > 0.8):
		move_msg.linear.x = 0.25
		twist_publisher.publish(move_msg)

	ang = -95
	if dist <= 0.8:
		ang = -105

	turn(twist_publisher,ang)

	start = time.time()
	while time.time() - start < 4:
		move_msg.linear.x = 0.25
		twist_publisher.publish(move_msg)

# STEP9 : Moving forward in the lobby , until reaching the pole
def step9(twist_publisher):
	global ranges

	move_msg = Twist()
	while anyMinus():
		move_msg.linear.x = 0.25
		twist_publisher.publish(move_msg)

# Check if at least one of the points belongs to the right range indicating the pole (the first one)
def anyMinus():
	global ranges
	tmp = ranges[0:8]
	for i in tmp:
		if i != -1:
			return False
	return True

# STEP10 : Moving towards the elevator direction, keeping away from the second pole
def step10(twist_publisher):
	global dist

	move_msg = Twist()
	start = time.time()
	while time.time() - start < 5:
		move_msg.linear.x = 0.25
		twist_publisher.publish(move_msg)

	turn(twist_publisher,-45)
	
	start = time.time()
	while time.time() - start < 4 and (dist > 1 or math.isnan(dist)):
		move_msg.linear.x = 0.25
		twist_publisher.publish(move_msg)

	turn(twist_publisher, 45)
	start = time.time()
	while time.time() - start < 6.5:
		move_msg.linear.x = 0.25
		twist_publisher.publish(move_msg)

# STEP11 : Turning left while looking for the red object (the elevator button)
def step11(twist_publisher):
	red_size = 0
	while red_size < 300:
		time.sleep(1)
		turn(twist_publisher,-0.1)
		time.sleep(2)
		red_size = red_object()[1]

# STEP12 : Moving towards the elevator (to the red object)
def step12(twist_publisher):
	move_until(twist_publisher,1)

# STEP13 : Make sure the robot is in the right position - infront of the red botton, and printing its location
def step13(twist_publisher):
	red_size = 0
	while red_size == 0:
		time.sleep(1)
		turn(twist_publisher, 0.1)
		time.sleep(2)
		red_size = red_object()[1]

	time.sleep(3)
	red_obj = red_object()
	print "location of red = " , red_obj[0]

# Move forward the given distance
def move_forward(twist_publisher, distance):
	global dist,min_dist

	move_msg = Twist()
	orig_dist = dist
	while abs(dist - orig_dist) < distance-0.05:
		if min_dist < 0.5:
			break
		move_msg.linear.x = 0.2
		twist_publisher.publish(move_msg)

	time.sleep(1)

# Move until the robot is far from the given distance
def move_until(twist_publisher, distance):
	global dist,min_dist

	move_msg = Twist()
	while dist > distance:
		move_msg.linear.x = 0.2
		twist_publisher.publish(move_msg)

	time.sleep(1)

# Turning according to the given degrees
def turn(twist_publisher,degrees):
	rads = math.radians(degrees)

	turn_msg = Twist()
	turn_msg.angular.z = -0.3 * (degrees / abs(degrees))
	current_time = rospy.Time.now().to_sec()
	while (rospy.Time.now().to_sec() - current_time < abs(rads) * 3):
		twist_publisher.publish(turn_msg)

# Looking for the biggest red object the robot sees, and returning its shape and its location
# If no red object is in the robot sight returning zero for both
def red_object():
	global pc2_msg

	while(pc2_msg==-1):
		time.sleep(1)

	rgb_list = []
	xyz_list = []

	for i in xrange(0, pc2_msg.width * pc2_msg.height):
	    bgr = map(lambda z: int(binascii.hexlify(z),16), struct.unpack("ccc", pc2_msg.data[i*32+16:i*32+19]))
	    rgb_list.append(bgr)
	    xyz = struct.unpack("<fff", pc2_msg.data[i*32:i*32+12])
	    xyz_list.append(xyz)
	    
	a = np.array(rgb_list)
	n = a.reshape([-1, pc2_msg.width, 3])

	xyz_list = np.array(xyz_list)
	xyz_list = xyz_list.reshape([-1, pc2_msg.width, 3])

	hsv_point = cv2.cvtColor(np.uint8(n), cv2.COLOR_BGR2HSV) 
	red_range = cv2.inRange(hsv_point, (160,100,100), (180,255,255))
	# cv2.imwrite("r1.jpg",red_range)

	(cnts, _) = cv2.findContours(red_range, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	max_red_size = 0
	max_red = 0
	for c in cnts:	
		xs = [x[0][1] for x in c]
		ys = [y[0][0] for y in c]	
		width = abs(min(xs)-max(xs))
		length = abs(min(ys)-max(ys))
		size =width*length
		if(size > max_red_size):
			max_red_size = size
			max_red = zip(xs,ys)

	if max_red == 0:
		return [0 , 0]

	red_xyz = 0
	for x, y in max_red:
		if not math.isnan(xyz_list[x,y][0]) and not math.isnan(xyz_list[x,y][2]):
			red_xyz = xyz_list[x,y]
			break

	return [red_xyz , max_red_size]


# MAIN PROGRAM
if __name__ == '__main__':
    try:
    	rospy.init_node('ass4', anonymous=True)
    	twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    	rospy.Subscriber("scan", LaserScan, update_ranges)
    	rospy.Subscriber("torso_camera/depth_registered/points", PointCloud2, update_image)

    	time.sleep(2)
    	step1(twist_publisher)
    	step2(twist_publisher)
    	step3(twist_publisher)
    	step4(twist_publisher)
    	step5(twist_publisher)
    	time.sleep(2)
    	step6(twist_publisher)
    	time.sleep(2)
    	step7(twist_publisher)
    	time.sleep(2)
    	step8(twist_publisher)
    	time.sleep(1)
    	step9(twist_publisher)
    	time.sleep(3)
    	step10(twist_publisher)
    	time.sleep(2)
    	step11(twist_publisher)
    	time.sleep(3)
    	step12(twist_publisher)
    	time.sleep(3)
    	step13(twist_publisher)

    	rospy.spin()

    except Exception as e:
    	print e
        pass




