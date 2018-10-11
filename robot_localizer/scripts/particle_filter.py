#!/usr/bin/env python

"""
ROS node for the particle filter
Authors: Anil Patel and Cedric Kim
"""

from __future__ import print_function, division
from geometry_msgs.msg import PointStamped, PointStamped, Twist
from neato_node.msg import Bump
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from scipy.stats import norm
from numpy.random import randn, random_sample
from datetime import datetime
from occupancy_field import OccupancyField
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import time, math, rospy
import random
from helper_functions import TFHelper

""" some global tuning knobs"""
#num_parts = 10

of = OccupancyField()
class ParticleFilter(object):
	"""particle filtering of a neato's location in a known map"""

	def __init__(self):
		"""initialize node, ROS things, etc."""
		rospy.init_node("ParticleFilter")
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.pub2 = rospy.Publisher('Robot_marker', Marker, queue_size = 10)
		self.pub3 = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 10)
		rospy.Subscriber('/odom', Odometry, self.read_pos)
		
		self.rate = rospy.Rate(20)
		self.xs_bl = None   # list of xs from lidar in base link frame
		self.ys_bl = None   # list of ys from lidar in base link frame
		self.particle_array = None  # list of particle objects, initialized later
		self.last_pose = None
		self.pose = None   #as a Twist
		self.orientation = None #as a quaternion
		self.tf = TFHelper()
		self.real_pose = None
		self.particle_array = []
		self.num_particles = 5
		self.sd_filter = self.num_particles/3.0 #standard deviation for random sampling
		self.current_pose = (0,0,0)
		self.previous_pose = (0,0,0)
		self.previous_time = rospy.get_time()
		self.current_time = rospy.get_time()
		self.initialize_pf()
		print('Init complete')

	def process_scan(self, message):
		"""take in scan data, returns x and y values in baselink ref frameself.
		   omits points with 0 range value."""
		ranges = message.ranges
		xs = []
		ys = []
		for i in range(len(ranges)):
			if ranges[i] != 0:
				theta = math.radians(i+90)
				r = ranges[i]
				xf = math.cos(theta)*r
				yf = math.sin(theta)*r
				xs.append(xf)
				ys.append(yf)

		# store vals in pf object attributes
		self.xs_bl = xs
		self.ys_bl = ys
	
	def initialize_pf(self):
		"""intialize the particle filter, called once"""
		for i in range(self.num_particles):
			x_pos = (random.randint(1,100) - 50)/10.0
			y_pos = (random.randint(1,100) - 50)/10.0
			theta = random.randint(1,360)
			theta = 0
			particle = Particle(x_pos, y_pos, theta, i)
			particle.weight = random.randint(1,100)
			self.particle_array.append(particle)

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
	
	def get_pose(self):
		"""gets current pose based on motor model"""
		return self.real_pose

	def run_points(self):
		"""runs all the important functions for point cloud evaluation and
		   propogation."""

		# define a "current" laser scan to loop through
		scanx_now = self.xs_bl
		scany_now = self.ys_bl

		# update all the particle weights and cumulative weight
		for particle in self.particle_array:
			particle.particles_to_map(scanx_now, scany_now)
			particle.get_particle_weight()

		# normalize weights
		#for particle in self.particle_array:
		#	particle.weight = particle.weight/total_weight

		# set a delta position from motor model
		#dist = self.pose_delta()


	def update_markers(self):
		"updates all particle markers"
		self.markerArray = MarkerArray()
		for particle in self.particle_array:
			x_pos = particle.x
			y_pos = particle.y
			#print(x_pos, y_pos)
			#if(self.real_pose != None):
			#	x_pos = self.real_pose[0]
			#	y_pos = self.real_pose[1]
			self.create_particle_marker(x_pos, y_pos)
			self.marker.id = particle.id
			self.markerArray.markers.append(self.marker)

	def sort_particles_by_weight(self):
		"sorts particles by their weight, normalizes weight, and id is redone"
		total_weight = sum(particle.weight for particle in self.particle_array)
		print('total_weight')
		print(total_weight)
		if(total_weight > 0):
			for particle in self.particle_array:
				print('here')
				print(particle.weight)
		self.particle_array.sort(key=lambda x: x.weight, reverse = True)


	def resample_particles(self, dx, dy, dtheta):
		"takes current odom reading and updates particles with noise"
		#print('here')
		new_array = []
		for i in range(self.num_particles):
			sample_id = self.num_particles
			while(sample_id >= self.num_particles):
				sample = np.random.normal(loc = 0.0, scale = self.sd_filter)
				sample_id = int(abs(sample))
				#print(sample_id)
			moved_theta = self.particle_array[sample_id].w + dtheta
			moved_x = self.particle_array[sample_id].x + dx*math.cos(math.radians(moved_theta))
			moved_y = self.particle_array[sample_id].y + dy*math.sin(math.radians(moved_theta))
			noisy_particle = self.particle_array[sample_id].return_particle_with_noise(moved_x, moved_y, moved_theta, i)
			new_array.append(noisy_particle)
		self.particle_array = new_array

	def create_particle_marker(self, x,y):
		"creates marker with position x,y"
		self.marker = Marker()
		self.marker.header.frame_id = "map"
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
		x = self.real_pose[0]
		y = self.real_pose[1]
		theta = math.degrees(self.real_pose[2])
		#print(self.real_pose[2], theta)
		#print(theta)
		self.real_pose = (x,y,theta)

	def create_robot_marker(self, real_pose, pose):
		self.robot_marker = Marker()
		self.robot_marker.header.frame_id = "map"
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
		self.robot_marker.color.b = 1

	def run(self):
		while not rospy.is_shutdown():
			if(self.real_pose != None):
				self.current_time = rospy.get_time()
				if(self.xs_bl != None):
					self.run_points()
					self.create_robot_marker(self.real_pose, self.pose)
					self.update_markers()
					self.sort_particles_by_weight()
					self.current_pose = self.get_pose()
					if(self.current_time - self.previous_time > .1):
						dx = self.current_pose[0] - self.previous_pose[0]
						dy = self.current_pose[1] - self.previous_pose[1]
						dtheta = self.current_pose[2] - self.previous_pose[2]
						#self.resample_particles(dx, dy, dtheta)
						self.previous_time = self.current_time
						self.previous_pose = self.get_pose()
					self.pub2.publish(self.robot_marker)
					self.pub3.publish(self.markerArray)
			self.rate.sleep()


class Particle(object):
	"""particle class for specific attributes and methods that represent the
	   sensor model."""

	def __init__(self, x_pos, y_pos, orient, id):
		self.id = id            # particle ID for tracking
		self.x = 4#x_pos          # particle pose
		self.y = -.15#y_pos
		self.w = 260#orient
		self.x_map = None
		self.y_map = None
		self.weight = None
		self.sd_position = .1/3 #standard deviation of position noise
		self.sd_theta = 2/3.0 #standard deviation (bounded by 5 degrees) of theta noise
		

	def particles_to_map(self, xs, ys):
		"""takes laser scan in base link at specific particle pose and converts
		   to xs and ys in the map coordinate frame."""

		# length x 2 array of all laser scan points
		lsr = np.array([xs, ys])
		lsr = np.transpose(lsr)

		# negate orientation and rotate
		theta = -math.radians(self.w)
		c, s = math.cos(theta), math.sin(theta)
		rotate = np.array([[c, -s], [s, c]])
		lsr_rot = np.dot(lsr, rotate)

		# negate position and translate
		x_move, y_move = -self.x, -self.y
		x_moved = lsr[:,0]+x_move
		y_moved = lsr[:,1]+y_move

	   	# final array of points, unused for now but maybe useful
	   	parts_map = np.array([x_moved, y_moved])

		# add x_map and y_map to the particle attributes
		self.x_map = x_moved
		self.y_map = y_moved

	def get_particle_weight(self):
		"""gets the particle's weight by comparing laser scan to map."""
		# loop through points
		weight = 0
		nan_count = 0
		for i in range(len(self.x_map)):
			x = self.x_map[i]
			y = self.y_map[i]
			dist = of.get_closest_obstacle_distance(x,y)
			print('dist')
			print(dist)
			# check for nan values
			if (not math.isnan(dist) and dist != 0):
				nan_count += 1      # track number of non-nan points
				weight += 1/(dist*1.0)    # weight inverse to distance

		# store the particle weight into an attribute
		print('nan_count')
		print(nan_count)
		if(nan_count != 0):
			self.weight = weight/(nan_count*1.0)  # normalize to non-nan scan points
		else:
			self.weight = 0
		
	def update_particles(self):
		"""update the position and orientation of the particles based on the
		   motion model

		make sure angles work out, for when you add past 360 degrees
		"""

	def run_particle(self):
		"""run all the particle functions we want to do for each particle for
		   simplicity"""
	
	def return_particle_with_noise(self, x_pos, y_pos, theta, id):
		"returns the particle with updated position and added noise"
		x_noise = np.random.normal(loc = x_pos, scale = self.sd_position)
		y_noise = np.random.normal(loc = y_pos, scale = self.sd_position)
		theta = np.random.normal(loc = theta, scale = self.sd_theta)
		particle = Particle(x_noise, y_noise, theta, id)
		return particle

if __name__ == '__main__':
	node = ParticleFilter()
	node.run()

