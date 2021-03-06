#!/usr/bin/env python

"""
ROS node for the particle filter
Authors: Anil Patel and Cedric Kim
"""

from __future__ import print_function, division
from geometry_msgs.msg import PointStamped, PointStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point32
from sensor_msgs.msg import PointCloud
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
		self.particle_pub = rospy.Publisher("particle_cloud", PointCloud, queue_size=10)
		rospy.Subscriber('/odom', Odometry, self.read_pos)
		self.rate = rospy.Rate(20)
		self.xs_bl = []   # list of xs from lidar in base link frame
		self.ys_bl = []   # list of ys from lidar in base link frame
		self.scanx_now = []
		self.scany_now = []
		self.last_pose = None
		self.pose = None   #as a Twist
		self.orientation = None #as a quaternion
		self.tf = TFHelper()
		self.real_pose = None
		self.particle_array = []
		self.num_particles = 500 #adjust to increase number of particles
		self.update_rate = .5 #in seconds
		self.sd_filter = self.num_particles*3.0/(4) #standard deviation for random sampling
		self.current_pose = (0,0,0)
		self.previous_pose = (0,0,0)
		self.previous_time = rospy.get_time()
		self.current_time = rospy.get_time()
		self.cloud_array = PointCloud() #point cloud for laser scan visualization
		self.x_array = []
		self.y_array = []
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
			#randomize x and y positions in a 5x5 meter square
			x_pos = (random.randint(1,100) - 50)/10.0
			y_pos = (random.randint(1,100) - 50)/10.0
			theta = random.randint(1,360)	#randomize initial theta
			particle = Particle(x_pos, y_pos, theta, i)
			self.particle_array.append(particle)

	def get_pose(self):
		"""gets current pose based on motor model"""
		return self.real_pose

	def create_cloud_array(self, x_array, y_array):
		"turns x and y laser points into cloud_array"
		cloud_array =[]
		for i in range(len(x_array)):
			x = x_array[i]
			y = y_array[i]
			z = 0
			point = Point32()
			point.x = x
			point.y = y
			point.z = 0
			cloud_array.append(point)

		self.cloud_array.header.frame_id = 'map'
		self.cloud_array.header.stamp = rospy.Time.now()
		self.cloud_array.points = cloud_array

	def map_odom_transform(self, pose):
		"converts map to odom coordinates"
		self.tf.fix_map_to_odom_transform(pose, rospy.Time.now())

	def update_markers(self):
		"updates all particle markers - only show the first 10"
		self.markerArray = MarkerArray()
		#if we have particles
		if(self.particle_array != None and len(self.particle_array) > 0):
			for i in range(10):
				particle = self.particle_array[i]
				x_pos = particle.x
				y_pos = particle.y
				#convert particle angle to marker quaternian 
				quaternion = self.tf.convert_angle_to_quaternion(particle.w)
				self.create_particle_marker(x_pos, y_pos, quaternion)
				self.marker.id = particle.id
				#for the highest weight particle, change color and size of arrow
				if(particle.id == 0):
					self.marker.color.b = 1
					self.marker.color.g = 0
					self.marker.scale.x = 1
					#map robot marker to particle with highest weight
					self.create_robot_marker(x_pos, y_pos, quaternion)
				self.markerArray.markers.append(self.marker)

	def sort_particles_by_weight(self):
		"sorts particles by their weight, normalizes weight, and id is reset"
		#sorts particles by weight
		self.particle_array.sort(key=lambda x: x.weight, reverse = True)
		#gets total weight of particles
		total_weight = sum(particle.weight for particle in self.particle_array)
		if(total_weight > 0):
			#normalizes weight and resets particle id
			for i, particle in enumerate(self.particle_array):
				particle.weight = particle.weight/total_weight
				particle.id = i
		#gets first particle laser scan
		if(len(self.particle_array) != None):
			self.x_array = self.particle_array[0].x_map
			self.y_array = self.particle_array[0].y_map


	def resample_particles(self, dx, dy, dtheta):
		"takes current odom reading and updates particles with noise"
		if(self.particle_array != None and len(self.particle_array) > 0):
			new_array = []
			#sd weight increases standard deviation if weight is lower
			sd_weight = 1 - self.particle_array[0].weight**(1/4)
			ran = False
			for i in range(self.num_particles):
				sample_id = self.num_particles
				#sample_id is a randomly choosen particle with a distribution 
				while(sample_id >= self.num_particles):
					sample = np.random.normal(loc = 0.0, scale = sd_weight*self.sd_filter)
					sample_id = int(abs(sample))
				if(not ran):
					sample_id = 0
				#moves particle with reference to the odom movement
				angle_bot = math.degrees(math.atan2(dy, dx))
				dist = math.sqrt(dx**2 + dy**2)
				current_angle = self.particle_array[sample_id].w
				moved_theta = self.particle_array[sample_id].w + dtheta
				cos_theta = math.cos(math.radians(current_angle))
				sin_theta = math.sin(math.radians(current_angle))
				moved_x = self.particle_array[sample_id].x + dist*cos_theta
				moved_y = self.particle_array[sample_id].y + dist*sin_theta
				#after moving particle, adds noise to particle
				if(ran):
					noisy_particle = self.particle_array[sample_id].return_particle_with_noise(moved_x, moved_y, moved_theta, i, sd_weight)
				else:
					noisy_particle = self.particle_array[sample_id]
					ran = True
				new_array.append(noisy_particle)
			self.particle_array = new_array

	def create_particle_marker(self, x,y, quaternion):
		"creates marker with position x,y"
		self.marker = Marker()
		self.marker.header.frame_id = "map"
		self.marker.type = self.marker.ARROW
		self.marker.action = self.marker.ADD
		self.marker.pose.position.x = x
		self.marker.pose.position.y = y
		self.marker.pose.orientation.x = quaternion[0]
		self.marker.pose.orientation.y = quaternion[1]
		self.marker.pose.orientation.z = quaternion[2]
		self.marker.pose.orientation.w = quaternion[3]
		scale = .15
		self.marker.scale.x = .5#scale
		self.marker.scale.y = .05
		self.marker.scale.z = .05
		self.marker.color.a = 1
		self.marker.color.g = 1

	def read_pos(self, data):
		"reads position of robot"
		self.pose = data.pose.pose
		self.real_pose = self.tf.convert_pose_to_xy_and_theta(self.pose)
		x = self.real_pose[0]
		y = self.real_pose[1]
		theta = math.degrees(self.real_pose[2])
		self.real_pose = (x,y,theta)

	def create_robot_marker(self, x_pos, y_pos, quaternion):
		"creates the robot marker"
		self.robot_marker = Marker()
		self.robot_marker.header.frame_id = "map"
		self.robot_marker.type = self.robot_marker.CUBE
		self.robot_marker.pose.position.x = x_pos
		self.robot_marker.pose.position.y = y_pos
		self.marker.pose.orientation.x = quaternion[0]
		self.marker.pose.orientation.y = quaternion[1]
		self.marker.pose.orientation.z = quaternion[2]
		self.marker.pose.orientation.w = quaternion[3]
		scale = .25
		self.robot_marker.scale.x = scale
		self.robot_marker.scale.y = scale
		self.robot_marker.scale.z = scale
		self.robot_marker.color.a = 1
		self.robot_marker.color.b = 1
	

	def run_points(self):
		"""runs all the important functions for point cloud evaluation and
		   propogation."""
		for particle in self.particle_array:
			particle.particles_to_map(self.scanx_now, self.scany_now)
			particle.get_particle_weight()

	def run(self):
		"runs particle filter"
		while not rospy.is_shutdown():
			if(self.xs_bl != None and self.pose != None and self.real_pose != None):
				self.current_time = rospy.get_time()
				if(self.current_time - self.previous_time > self.update_rate):
					self.current_pose = self.get_pose()
					dx = self.current_pose[0] - self.previous_pose[0]
					dy = self.current_pose[1] - self.previous_pose[1]
					dtheta = self.current_pose[2] - self.previous_pose[2]
					self.resample_particles(dx, dy, dtheta)
					self.previous_time = rospy.get_time()
					self.previous_pose = self.get_pose()
					self.scanx_now = self.xs_bl
					self.scany_now = self.ys_bl

				self.run_points()
				self.sort_particles_by_weight()
				self.update_markers()
				self.tf.send_last_map_to_odom_transform()
				self.map_odom_transform(self.pose)
				self.create_cloud_array(self.x_array, self.y_array)

				#publish things
				self.particle_pub.publish(self.cloud_array)
				self.pub2.publish(self.robot_marker)
				self.pub3.publish(self.markerArray)
			self.rate.sleep()


class Particle(object):
	"""particle class for specific attributes and methods that represent the
	   sensor model."""

	def __init__(self, x_pos, y_pos, orient, id):
		self.id = id            # particle ID for tracking
		self.x = x_pos          # particle pose
		self.y = y_pos
		self.w = orient
		self.x_map = None
		self.y_map = None
		self.weight = 0
		self.sd_position = .2/3 #standard deviation of position noise
		self.sd_theta = 90/3.0 #standard deviation of theta noise

	def particles_to_map(self, xs, ys):
		"""takes laser scan in base link at specific particle pose and converts
		   to xs and ys in the map coordinate frame."""

		# length x 2 array of all laser scan points
		lsr = np.array([xs, ys])
		lsr = np.transpose(lsr)

		# negate orientation and rotate
		theta = -math.radians(self.w) + math.radians(90)
		c, s = math.cos(theta), math.sin(theta)
		rotate = np.array([[c, -s], [s, c]])
		lsr_rot = np.dot(lsr, rotate)

		# negate position and translate
		x_move, y_move = self.x, self.y
		x_moved = lsr_rot[:,0]+x_move
		y_moved = lsr_rot[:,1]+y_move

	   	# final array of points, unused for now but maybe useful
	   	parts_map = np.array([x_moved, y_moved])

		# add x_map and y_map to the particle attributes
		self.x_map = x_moved
		self.y_map = y_moved

	def get_particle_weight(self):
		"""gets the particle's weight by comparing laser scan to map."""
		# loop through points
		weight = 0
		sum_dist = 0
		nan_count = 0
		for i in range(len(self.x_map)):
			x = self.x_map[i]
			y = self.y_map[i]
			dist = of.get_closest_obstacle_distance(x,y)
			# check for nan values
			if (not math.isnan(dist) and dist != 0):
				nan_count += 1      # track number of non-nan points
				sum_dist += dist    # weight inverse to distance
		if(sum_dist != 0):
			weight = 1.0/sum_dist
		# store the particle weight into an attribute
		if(nan_count != 0):
			self.weight = weight*(nan_count*1.0)  # normalize to non-nan scan points
		else:
			self.weight = 0
	
	def return_particle_with_noise(self, x_pos, y_pos, theta, id, sd_weight):
		"returns the particle with updated position and added noise"
		x_noise = np.random.normal(loc = x_pos, scale = sd_weight*self.sd_position)
		y_noise = np.random.normal(loc = y_pos, scale = sd_weight*self.sd_position)
		theta = np.random.normal(loc = theta, scale = sd_weight*self.sd_theta)
		particle = Particle(x_noise, y_noise, theta, id)
		return particle

if __name__ == '__main__':
	node = ParticleFilter()
	node.run()

