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

class ParticleFilter(object):
    """particle filtering of a neato's location in a known map"""

    def __init__(self):
        """initialize node, ROS things, etc."""
        rospy.init_node("ParticleFilter")
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.xs_bl = None       # list of xs from lidar in base link frame
        self.ys_bl = None       # list of ys from lidar in base link frame
        self.particles = None   # list of particle objects, initialized later


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
        # initialize all particle objects and assign to self.particles
        #
    def get_weights(self):
        """update the weights at a particular timestep"""

    def normalize_weights(self):
        """normalize weights for sampling"""

    def propogate_points(self):
        """uses the motor model to move the particles to new values. the motor
           model will have important things like adding noise. we'll have to
           feed it time steps to track odom in to get the right delta pos"""


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


    def particles_to_map(self, xs, ys):
        """takes laser scan in base link at specific particle pose and converts
           to xs and ys in the map coordinate frame."""

           # length x 2 array of all laser scan points
           lsr = np.array([xs, ys])
           lsr = np.transpose(lsr)

           # negate orientation and rotate
           theta = -self.rad
           c, s = math.cos(theta), math.sin(theta)
           rotate = np.array([[c, -s], [s, c]])
           lsr_rot = np.dot(lsr, rotate)

           # negate position and translate
           x_move, y_move = -self.x, -self.y
           x_moved = lsr[:,0]+x_move
           y_moved = lsr[:,1]+y_move

           # final array of points, unused for now but maybe useful
           parts_map = np.array([x_moved, y_moved])
           return None


    def get_particle_weight(self):
        """gets the particle's weight by comparing laser scan to map."""

        self.particles_to_map()     # map all particles with latest scan data

        # loop through points
        weight = 0
        for i in range(length(self.x_map)):
            x = self.x_map[i]
            y = self.y_map[i]
            dist = of.get_closest_obstacle_distance(x,y)
            # check for nan values
            if not isnan(dist):
                sub_weights.append(1/dist)

        # store the particle weight into its attribute
        self.weight = weight

    def update_particles(self):
        """ update the position and orientation of the particles based on the
            motion model

        make sure angles work out, for when you add past 360 degrees

        """

    def run_particle(self):
        """run all the particle functions we want to do for each particle for
           simplicity"""
