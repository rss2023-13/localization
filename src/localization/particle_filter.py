#!/usr/bin/env python2

import rospy
import tf.transformations as tf
from sensor_model import SensorModel
from motion_model import MotionModel
import numpy as np
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import threading
from std_msgs.msg import Float32MultiArray
import math


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame", "/base_link_pf")
        self.rate = 26 #hertz
        self.flag = 0 

        self.tfBuffer = tf2_ros.Buffer()
	    self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.lidar_callback,
                                          queue_size=1)

        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odom_callback,
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_initialization,
                                          queue_size=1)

        self.error_pub = rospy.Publisher("/error", Float32MultiArray)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

        self.particle_pub  = rospy.Publisher("/particles", PoseArray, queue_size = 1)

        self.num_particles = rospy.get_param("~num_particles", 200)
        self.particles = np.zeros((self.num_particles, 3))
        self.lock = threading.Lock() # for particle array self.particles

        self.probs = np.ones((self.num_particles,))
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

       

    # Implement the MCL algorithm
    # using the sensor model and the motion model
    #
    # Make sure you include some way to initialize
    # your particles, ideally with some sort
    # of interactive interface in rviz
    #
    # Publish a transformation frame between the map
    # and the particle_filter_frame.

    def publish_particles(self):
        particle_array = PoseArray()
        particle_list = []
        particle_array.header.frame_id = "map"
        for particle in self.particles:
            point = Point()
            point.x = particle[0]
            point.y = particle[1]
            point.z = 0

            orientation = Quaternion()
            q = tf.quaternion_from_euler(0,0,particle[2])
            orientation.x = q[0]
            orientation.y = q[1]
            orientation.z = q[2]
            orientation.w = q[3]

            pose = Pose()
            pose.position = point
            pose.orientation = orientation

            particle_list.append(pose)

        particle_array.poses = particle_list
        self.particle_pub.publish(particle_array)


    def publish_average_point(self, particles, probs):
        probs = probs ** 2
        new_x = np.average(particles[:,0], weights=probs)
        new_y = np.average(particles[:,1], weights=probs)

        # rospy.loginfo(particles[0])

        angles = self.particles[:,2]
        angle_vecs = np.array([np.cos(angles), np.sin(angles)])
        new_point = angle_vecs.mean(axis=1)
        new_angle = np.arctan2(new_point[1], new_point[0])

        point = Point()
        point.x = new_x
        point.y = new_y
        point.z = 0

        orientation = Quaternion()
        q = tf.quaternion_from_euler(0,0,new_angle)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]

        odom_msg = Odometry()
        odom_msg.child_frame_id = self.particle_filter_frame
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose.position = point
        odom_msg.pose.pose.orientation = orientation
        odom_msg.header.stamp = rospy.Time.now()

        self.odom_pub.publish(odom_msg)

        ########## GRAPHING ERROR ######################

        time_error_msg = Float32MultiArray()

        transform = tfBuffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(1.0))
        real_x = transform.transform.translation.x
        real_y = transform.transform.translation.y

        error_dist = math.sqrt((real_x - point.x)**2 + (real_y - point.y)**2)
        time_error_msg.data = [error_dist]
        self.error_pub.publish(time_error_msg)

    def lidar_callback(self, lidar_data):

        probs = np.array(self.sensor_model.evaluate(self.particles, lidar_data.ranges)) #CHANGE THIS LATER, VECTORIZE STUFF IN SENSOR MODEL
        self.probs = probs

        if self.flag%3 == 0:     

            probs = probs ** .75
            
            self.lock.acquire()
            self.particles = self.particles[np.random.choice(np.arange(self.num_particles), size=self.num_particles, p=probs/probs.sum())]
            self.lock.release()

               
            # Publish the "average pose" of the particles
            # TODO: Experiment with the weighted average
            self.publish_average_point(self.particles, self.probs)
            self.publish_particles()

        self.flag += 1

    def odom_callback(self, odom_data):

        # First get the new particles from the motion model
        linear = odom_data.twist.twist.linear
        angular = odom_data.twist.twist.angular

        dx = linear.x/self.rate
        dy = linear.y/self.rate
        dtheta = angular.z/self.rate

        # rospy.loginfo(linear)
        self.lock.acquire()
        self.particles = self.motion_model.evaluate(self.particles, np.array([dx, dy, dtheta]))
        self.lock.release()

        # Get the "average" particle through a weighted average
        self.publish_average_point(self.particles, self.probs)
        self.publish_particles()

        
    
    def pose_initialization(self, pose_data):

        position = pose_data.pose.pose.position
        q = pose_data.pose.pose.orientation
        
        
        angles = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])

        base_point = [position.x, position.y, angles[2]]
        particles = np.zeros((self.num_particles, 3))

        particles[:,1] += base_point[1]
        particles[:,2] += base_point[2]

        scale = 0.05

        particles[:, 0] = np.random.normal(loc=position.x, scale=.3, size=self.num_particles)
        particles[:, 1] = np.random.normal(loc=position.y, scale=.3, size=self.num_particles)
        particles[:, 2] = np.random.normal(loc=angles[2], scale=abs(scale*angles[2]), size=self.num_particles)
        
        self.lock.acquire()
        self.particles = particles
        self.lock.release()

        self.probs = np.ones(self.num_particles)/self.num_particles



if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
