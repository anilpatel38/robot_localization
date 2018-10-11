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
import numpy as np
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
		self.num_particles = 15
		self.create_particle_array()
		self.sd_filter = self.num_particles/3.0 #standard deviation for random sampling
		self.sd_position = .1 #standard deviation of position noise
		self.sd_theta = 5/3.0 #standard deviation (bounded by 5 degrees) of theta noise

	def create_particle_array(self):
		"creates initial particles"
		self.particle_array = []
		for i in range(self.num_particles):
			x_pos = random.randint(1,100)
			x_pos = x_pos/100.0
			y_pos = random.randint(1,100)
			y_pos = y_pos/100.0
			particle = Particle(x_pos, y_pos, 1)
			particle.id = i
			particle.weight = random.randint(1,100)
			self.particle_array.append(particle)

	def update_markers(self):
		"updates all particle markers"
		self.markerArray = MarkerArray()
		for particle in self.particle_array:
			x_pos = particle.x
			y_pos = particle.y
			print(x_pos, y_pos)
			#if(self.real_pose != None):
			#	x_pos = self.real_pose[0]
			#	y_pos = self.real_pose[1]
			self.create_particle_marker(x_pos, y_pos)
			self.marker.id = particle.id
			self.markerArray.markers.append(self.marker)

	def sort_particles_by_weight(self):
		"sorts particles by their weight, normalizes weight, and id is redone"
		total_weight = sum(particle.weight for particle in self.particle_array)
		if(total_weight > 0):
			for particle in self.particle_array:
				particle.weight = particle.weight/(total_weight*1.0)
		self.particle_array.sort(key=lambda x: x.weight, reverse = True)
		for i in range(self.num_particles):
			self.particle_array[i].id = i

	def resample_particles(self, x_pos, y_pos, theta):
		"takes current odom reading and updates particles with noise"
		new_array = []
		for i in range(self.num_particles):
			sample_id = self.num_particles
			while(sample_id >= self.num_particles):
				sample = np.random.normal(loc = 0.0, scale = self.sd_filter)
				sample_id = int(abs(sample))
			noisy_particle = self.return_particle_with_noise(self.particle_array[i], x_pos, y_pos, theta)
			noisy_particle.id = i
			new_array.append(noisy_particle)
		self.particle_array = new_array
		#print('start')
		#for particle in self.particle_array:
		#	print(particle.weight)
		
	def return_particle_with_noise(self, particle, x_pos, y_pos, theta):
		"returns the particle with updated position and added noise"
		x_noise = np.random.normal(loc = x_pos, scale = self.sd_position)
		y_noise = np.random.normal(loc = y_pos, scale = self.sd_position)
		theta = np.random.normal(loc = theta, scale = self.sd_theta)
		particle.x = x_noise
		particle.y = y_noise
		particle.w = theta
		return particle

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
		self.marker.color.g = 1

	def read_pos(self, data):
		self.pose = data.pose.pose
		self.real_pose = self.tf.convert_pose_to_xy_and_theta(self.pose)

	def create_robot_marker(self, real_pose, pose):
		self.robot_marker = Marker()
		self.robot_marker.header.frame_id = "odom"
		self.robot_marker.type = self.robot_marker.CUBE
		self.robot_marker.pose.position.x = real_pose[0]
		self.robot_marker.pose.position.y = real_pose[1]
		self.robot_marker.pose.orientation.x = self.pose.orientation.x
		self.robot_marker.pose.orientation.y = self.pose.orientation.y
		self.robot_marker.pose.orientation.z = self.pose.orientation.z
		self.robot_marker.pose.orientation.w = self.pose.orientation.w
		scale = .25
		self.robot_marker.scale.x = scale
		self.robot_marker.scale.y = scale
		self.robot_marker.scale.z = scale
		self.robot_marker.color.a = 1
		self.robot_marker.color.g= 1

	def run(self):
		while not rospy.is_shutdown():
			if(self.real_pose != None):
				self.create_robot_marker(self.real_pose, self.pose)
				self.update_markers()
				#self.sort_particles_by_weight()
				#self.resample_particles(1,1,1)
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

