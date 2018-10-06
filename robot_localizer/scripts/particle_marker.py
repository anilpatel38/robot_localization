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

class particle_marker(object):
	"""adds particles or something"""

	def __init__(self):
		rospy.init_node('particle_marker')
		self.rate = rospy.Rate(2)
		self.num_particles = 10
		self.particles = None
		self.pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 10)

	def create_array(self):
		self.particles = []
		for i in range(self.num_particles):
			x_pos = random.randint(1,100)
			x_pos = x_pos/50.0
			y_pos = random.randint(1,100)
			y_pos = y_pos/50.0
			self.particles.append((x_pos, y_pos))

	def update_markers(self):
		"updates all markers"
		self.markerArray = MarkerArray()
		id_number = 0
		for pos in self.particles:
			self.create_particle_marker(pos[0], pos[1])
			self.marker.id = id_number
			self.markerArray.markers.append(self.marker)
			id_number += 1

	def create_particle_marker(self, x,y):
		"creates marker with position x,y"
		self.marker = Marker()
		self.marker.header.frame_id = "base_link"
		self.marker.type = self.marker.SPHERE
		self.marker.action = self.marker.ADD
		self.marker.pose.position.x = x
		self.marker.pose.position.y = y
		self.marker.pose.position.z = 0
		scale = .15
		self.marker.scale.x = scale
		self.marker.scale.y = scale
		self.marker.scale.z = scale
		self.marker.color.a = 1
		self.marker.color.g= 1

	def update_current_location(self):
		pass

	def read_pos(self, data):
		self.pos = data.pose.pose.position
		self.orientation = data.pose.pose.orientation



	def run(self):
		while not rospy.is_shutdown():
			self.create_array()
			self.update_markers()
			self.pub.publish(self.markerArray)
			self.rate.sleep()

if __name__ == '__main__':
	node = particle_marker()
	node.run()