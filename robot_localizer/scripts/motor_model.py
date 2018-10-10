#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import random
from helper_functions import TFHelper

class motor_model(object):
	"""spits out distance from last point"""

	def __init__(self):
		rospy.init_node('motor_model')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.pub2 = rospy.Publisher('Robot_marker', Marker, queue_size = 10)
		self.pub3 = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 10)
		rospy.Subscriber('/odom', Odometry, self.read_pos)
		self.rate = rospy.Rate(10)
		self.key = None
		self.pose = None   #as a Twist
		self.orientation = None #as a quaternion
		self.tf = TFHelper()
		self.real_pose = None
		self.particle_array = []
		self.create_particle_array()

	def create_particle_array(self):
		"creates initial particles"
		self.particle_array = []
		for i in range(10):
			x_pos = random.randint(1,100)
			x_pos = x_pos/100.0
			y_pos = random.randint(1,100)
			y_pos = y_pos/100.0
			particle = Particle(x_pos, y_pos, 1)
			particle.id = i
			self.particle_array.append(particle)


	def update_markers(self):
		"updates all markers"
		self.markerArray = MarkerArray()
		for particle in self.particle_array:
			if(self.real_pose != None):
				x_pos = self.real_pose[0] + particle.x*math.cos(- (self.real_pose[2]))
				y_pos = self.real_pose[1] #+ particle.y#- particle.y*math.sin( (self.real_pose[2]))
				self.create_particle_marker(x_pos, y_pos)
			self.marker.id = particle.id
			self.markerArray.markers.append(self.marker)

	def sort_add_noise(self):
		pass
	def create_particle_marker(self, x,y):
		"creates marker with position x,y"
		self.marker = Marker()
		self.marker.header.frame_id = "odom"
		self.marker.type = self.marker.SPHERE
		self.marker.action = self.marker.ADD
		self.marker.pose.position.x = x
		self.marker.pose.position.y = y
		scale = .15
		self.marker.scale.x = scale
		self.marker.scale.y = scale
		self.marker.scale.z = scale
		self.marker.color.a = 1
		self.marker.color.g= 1

	def read_pos(self, data):
		self.pose = data.pose.pose
		self.real_pose = self.tf.convert_pose_to_xy_and_theta(self.pose)

	def create_robot_marker(self, pose):
		self.robot_marker = Marker()
		self.robot_marker.header.frame_id = "odom"
		self.robot_marker.type = self.robot_marker.CUBE
		self.robot_marker.pose.position.x = pose[0]
		self.robot_marker.pose.position.y = pose[1]
		scale = .25
		self.robot_marker.scale.x = scale
		self.robot_marker.scale.y = scale
		self.robot_marker.scale.z = scale
		self.robot_marker.color.a = 1
		self.robot_marker.color.g= 1

	def run(self):
		while not rospy.is_shutdown():
			if(self.real_pose != None):
				self.create_robot_marker(self.real_pose)
				self.update_markers()
				self.pub2.publish(self.robot_marker)
				self.pub3.publish(self.markerArray)
				self.rate.sleep()

class Particle(object):
    """particle class for specific attributes and methods that represent the
       sensor model."""

    def __init__(self, x_pos, y_pos, orient):
        self.id = None      # particle ID for tracking
        self.x = x_pos
        self.y = y_pos
        self.w = orient
        self.rad = math.radians(orient)
        self.x_map = None
        self.y_map = None
        self.weight = None

if __name__ == '__main__':
	node = motor_model()
	node.run()

