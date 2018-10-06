#!/usr/bin/env python

"""
ROS node for the particle filter
Authors: Anil Patel and Cedric Kim
"""
from __future__ import print_function, division
from geometry_msgs.msg import PointStamped, PointStamped, Twist
from std_msgs.msg import Header Float64
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from scipy.stats import norm
from numpy.random import randn, random_sample
from datetime import datetime
import statistics
import numpy as np
import time, math, rospy

class ParticleFilter(object):
    """particle filtering of a neato's location in a known map"""

    def __init__(self, sensor_model):
        """initialize node, ROS things, etc."""
        rospy.init_node("ParticleFilter")
        self.sensor_model = sensor_model
        # subscribe to odom?
        # rospy.Subscriber('/bump', Bump, self.process_bump)

class SensorModel(object):
    """sensor model for the laser scan"""

    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

        self.xs_bl = None   # list of xs from lidar in base link frame
        self.ys_bl = None   # list of ys from lidar in base link frame


    def process_scan(self, message):
        """take in scan data, returns x and y values in baselink ref frame"""
        ranges = m.ranges
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
        self.xs = xs
        self.ys = ys

class Particle(object):
    """particle class for specific attributes and methods that represent the
       sensor model."""

    def __init__(self, x_pos, y_pos, orient, sensor_model):
        self.x = x_pos
        self.y = y_pos
        self.w = orient
        self.rad = math.radians(orient)
        self.x_map = None
        self.y_map = None
        self.weight = None
        self.sensor_model = sensor_model

    def particles_to_map(self):
        """takes laser scan data local to the partcle's pose and converts into
           xs and ys in the map coordinate frame.

           updates particle attributes for x_map and y_map"""

           # length x 2 array of all laser scan points
           lsr = np.array([self.sensor_model.xs_bl, self.sensor_model.ys_bl])
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
        """take list of laser data from particle location in map coordinates

        define a subset of points to look at for computational speed
        for point in points:
        """

    def update_particles(self):
        """ update the position and orientation of the particles based on the
            motion model

        make sure angles work out, for when you add past 360 degrees

        """
