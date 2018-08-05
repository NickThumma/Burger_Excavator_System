#! /usr/bin/env python
#specifies to use python, not C++

import rospy
from geometry_msgs.msg import Twist
import requests
from math import pi, asin, sqrt
from time import sleep
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


##################################################################

	#"""ThingWorx helper functions"""

server = "https://academic-educatorsextension.portal.ptc.io/Thingworx/Things/"                                      
thing = "mobile_burger_hug/" 
appKey = "?appKey=bf80b928-ad7d-4040-9018-b3dc0b3b9527&method=post&x-thingworx-session=true"

def updateLastComm():
	service = "Services/updateLastComm"
	req = server+thing+service+appKey
	p = requests.get(req)


def tw_get_generic(service, propertyName):
	# Connecting to ThingWorx
	updateLastComm()
	service = "Services/" + service
	req = server+thing+service+appKey+"&propertyName="+propertyName
	p = requests.get(req)
	output = p.text[p.text.find("<TD>")+4:p.text.find("</TD>")]
	output = str(output).replace("&quot;","")
	try:
		output = float(output)
	except:
		pass
	return output


def tw_set_generic(service, value):
	# Connecting to ThingWorx
	updateLastComm()
	service = "Services/" + service
	req = server+thing+service+appKey+value
	p = requests.get(req)


	#get functions

def accelerationTime():
	service = "GetNumberPropertyValue"
	propertyName = "AccelerationTime"
	return tw_get_generic(service, propertyName)

def turningRadius():
	service = "GetNumberPropertyValue"
	propertyName = "TurningRadius"
	return tw_get_generic(service, propertyName)/1000 
	#ThingWorx rounds too much, so easiest to get data as integer and divide

def canMove():
	service = "GetBooleanPropertyValue"
	propertyName = "CanMove"
	if tw_get_generic(service,propertyName) == "true":
		return True
	else:
		return False

def status():
	service = "GetStringPropertyValue"
	propertyName = "Status"
	return tw_get_generic(service,propertyName)


	#set functions

def setStatus(value):
	service = "setStatus"
	value = "&status="+str(value)
	tw_set_generic(service, value)

def setSlopeAngle(value):
	service = "setSlopeAngle"
	value = "&slopeAngle="+str(value)
	tw_set_generic(service, value)


##################################################################

	#"""ROS helper classes"""

class OdometryData:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
		self.dist = 0
		self.subscriber = rospy.Subscriber("odom", Odometry, self.callback)

	def get_dist(self):
		self.dist = sqrt(self.x**2+self.y**2)
		return self.dist

	def callback(self,msgs):
		self.x = msgs.pose.pose.position.x
		self.y = msgs.pose.pose.position.y
		self.z = msgs.pose.pose.position.z


class AngularData:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
		self.w = 0
		self.euler = [0,0,0]
		self.subscriber = rospy.Subscriber("imu", Imu, self.callback)

	def callback(self,msgs):
		#orientation
		self.x = msgs.orientation.x
		self.y = msgs.orientation.y
		self.z = msgs.orientation.z
		self.w = msgs.orientation.w
		euler = tf.transformations.euler_from_quaternion((self.x,self.y,self.z,self.w))
		for i in range(3):
			self.euler[i] = euler[i]*180/pi #radians to degrees
			if euler[i] < 0:
				self.euler[i] += 360

##################################################################


def talker():
	#initialize variables used to communicate with robot
	rospy.init_node('vel_publisher')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	move = Twist()
	rate = rospy.Rate(100) #Hz
	angular_vel = .5
	max_speed = .19
	
	#initialize odometer
	loc = OdometryData()
	loc.subscriber

	#initialize gyroscope
	gyro = AngularData()
	gyro.subscriber

	#"""start helper functions"""

	def change_velocity(vel, ang): #change, not set because modifies existing velocity
		move.linear.x += vel
		move.angular.z += ang
		pub.publish(move)
		rate.sleep()

	def get_dist():
		loc.subscriber
		return loc.get_dist()

	def get_angle():
		gyro.subscriber
		return gyro.euler

	def turn(angle, right):
		initial_angle = get_angle()[2]

		if right:
			#direction = "Turn Right"
			v = -angular_vel
			angle = -angle
		else:
			#direction = "Turn Left"
			v = angular_vel

		#setStatus(direction)
		final_angle = initial_angle + angle
		if final_angle < 0:
			final_angle = (final_angle%360)

		change_velocity(0,v)
		while get_angle()[2] < (final_angle-4*angular_vel) or get_angle()[2] > (final_angle+4*angular_vel):
			sleep(.01)
		change_velocity(0,-v)

	#"""end helper functions"""

	setStatus("Idle")
	while not status() == "Go 1":
		sleep(.5)
	#print("Go 1")

	start = get_dist()
	initial_slope = get_angle()[1]
	forward = True
	change_velocity(.1,0)
	while not rospy.is_shutdown():
		if forward and abs(get_dist() - start) < .2:
			pass

	#		Measure angle and transfer data to thingworx
		elif forward:
			change_velocity(-.1,0)
			rospy.sleep(.5)

			#setup angular velocity and initial variables
			temp = angular_vel
			angular_vel = .6

			initial_angle = get_angle()[2]
			opposite = (initial_angle+180)%360
			all_slopes = [0]
			change_velocity(0,angular_vel)

			#measure angles
			while get_angle()[2] < (opposite-4*angular_vel) or get_angle()[2] > (opposite+4*angular_vel):
				if get_angle()[1] != all_slopes[-1]:
					all_slopes.append(abs(get_angle()[1] - initial_slope))
			while get_angle()[2] < (initial_angle-4*angular_vel) or get_angle()[2] > (initial_angle+4*angular_vel):
				pass
			
			#stop spinning
			change_velocity(0, -angular_vel)
			angular_vel = temp

			#find max angle from all measured angles
			max_slope = 0
			all_slopes = [min(i, 360-i) for i in all_slopes]
			current_slope = sum(all_slopes[0:20])
			for i in range(len(all_slopes) - 20):
				max_slope = max(max_slope, abs(current_slope))
				current_slope = current_slope - all_slopes[i] + all_slopes[i+20]
			max_slope /= 20 

			#transmit max angle and start going backwards
			setSlopeAngle(max_slope)
			start = get_dist()
			forward = False
			change_velocity(-.1,0)

		elif abs(get_dist() - start) < .2:
			pass

		else:
			change_velocity(.1,0)
			setStatus("Off") #triggers Excavator start
			break


	#wait until time to move
	while not status() == "Go 2" and not rospy.is_shutdown():
		sleep(.5)
	#print("Go 2")


	#get parameters from thingworx
	if canMove():
		acc = accelerationTime()
		radius = turningRadius() 
	else:
		print("Too heavy to move.")
		rospy.signal_shutdown("Done")


	#convert parameters to usable values
	max_speed = .19
	increment = max_speed*10/(acc) 
	angular_vel = 1.6/(radius*2*pi*1.316) #from radius data


	#move
	setStatus("Move")
	turn(90, True)
	# setStatus("Turn Right")
	# initial_angle = get_angle()[2]
	# change_velocity(0,-angular_vel)
	# while not rospy.is_shutdown() and abs(get_angle()[2] - initial_angle) < (90 - 4*angular_vel):
	# 	pass
	# change_velocity(0,angular_vel)


	#setStatus("Go Straight")
	start = get_dist()
	while not rospy.is_shutdown():
		if move.linear.x < .19:
			change_velocity(increment,0)
		elif abs(get_dist() - start) < .1:
			pass
		else:
			break

	turn(90, True)
	# setStatus("Turn Right")
	# initial_angle = get_angle()[2]
	# change_velocity(0,-angular_vel)
	# while not rospy.is_shutdown() and abs(get_angle()[2] - initial_angle) < 90:
	# 	pass
	# change_velocity(0,angular_vel)

	#setStatus("Go Straight")
	start = get_dist()
	while not rospy.is_shutdown():
		if abs(get_dist() - start) < .2:
			pass
		elif move.linear.x > 0:
			change_velocity(-increment,0)
		else:
			break

	move.linear.x = 0
	move.angular.z = 0
	pub.publish(move)
	setStatus("Done")
	rospy.signal_shutdown("Done")




if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		print('The program did not execute.')
