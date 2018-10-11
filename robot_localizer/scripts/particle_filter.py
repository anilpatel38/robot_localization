 #!/usr/bin/env python

"""
ROS node for the particle filter
Authors: Anil Patel and Cedric Kim
"""
from __future__ import print_function, division
from geometry_msgs.msg import PointStamped, PointStamped, Twist
from std_msgs.msg import Header Float64
from neato_node.msg import Bump
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from scipy.stats import norm
from numpy.random import randn, random_sample
from datetime import datetime
from occupancy_field import OccupancyField as of
import statistics
import numpy as np
import time, math, rospy

""" some global tuning knobs"""
num_parts = 10

class ParticleFilter(object):
    """particle filtering of a neato's location in a known map"""

    def __init__(self):
        """initialize node, ROS things, etc."""
        rospy.init_node("ParticleFilter")
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.xs_bl = None   # list of xs from lidar in base link frame
        self.ys_bl = None   # list of ys from lidar in base link frame
        self.particle_array = None  # list of particle objects, initialized later
        self.last_pose = None

        self.initialize_pf()

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
		for i in range(num_parts):
			x_pos = random.randint(1,100)/100.0
			y_pos = random.randint(1,100)/100.0
            theta = random.randint(1,360)
			particle = Particle(x_pos, y_pos, theta, id)
			self.particle_array.append(particle)

    def get_pose(self):
        """gets current pose based on motor model"""

        return dist

    def run_points(self):
        """runs all the important functions for point cloud evaluation and
           propogation."""

        # define a "current" laser scan to loop through
        scanx_now = self.xs_bl
        scany_now = self.ys_bl
        total_weight = 0

        # update all the particle weights and cumulative weight
        for particle in self.particle_array:
            particle.particles_to_map(scanx_now, scany_now)
            particle.get_particle_weight()
            total_weight += particle.weight

        # normalize weights
        for particle in self.particle_array:
            self.weight = self.weight/total_weight

        # set a delta position from motor model
        dist = self.pose_delta()




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
        self.weight = None

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
        for i in range(length(self.x_map)):
            x = self.x_map[i]
            y = self.y_map[i]
            dist = of.get_closest_obstacle_distance(x,y)
            # check for nan values
            if not isnan(dist):
                num_count += 1      # track number of non-nan points
                weight += 1/dist    # weight inverse to distance

        # store the particle weight into an attribute
        self.weight = weight/num_count  # normalize to non-nan scan points

    def update_particles(self):
        """update the position and orientation of the particles based on the
           motion model

        make sure angles work out, for when you add past 360 degrees
        """

    def run_particle(self):
        """run all the particle functions we want to do for each particle for
           simplicity"""
