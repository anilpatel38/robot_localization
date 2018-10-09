#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import rospy
import math
from helper_functions import TFHelper

class motor_model(object):
	"""spits out distance from last point"""

	def __init__(self):
		rospy.init_node('motor_model')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.pub2 = rospy.Publisher('Robot_marker', Marker, queue_size = 10)
		rospy.Subscriber('/odom', Odometry, self.read_pos)
		self.rate = rospy.Rate(10)
		self.key = None
		self.pose = None   #as a Twist
		self.orientation = None #as a quaternion
		self.tf = TFHelper()
		self.real_pose = None

	def update_current_location(self):
		pass
	def read_pos(self, data):
		self.pose = data.pose.pose
		self.real_pose = self.tf.convert_pose_to_xy_and_theta(self.pose)

	def create_robot_marker(self, pose):
		self.marker = Marker()
		self.marker.header.frame_id = "odom"
		self.marker.type = self.marker.CUBE
		self.marker.pose.position.x = pose[0]
		self.marker.pose.position.y = pose[1]
		self.marker.pose.position.z = 0
		scale = .25
		self.marker.scale.x = scale
		self.marker.scale.y = scale
		self.marker.scale.z = scale
		self.marker.color.a = 1
		self.marker.color.g= 1

	def run(self):
		while not rospy.is_shutdown():
			if(self.real_pose != None):
				self.create_robot_marker(self.real_pose)
				self.pub2.publish(self.marker)
				self.rate.sleep()

if __name__ == '__main__':
	node = motor_model()
	node.run()