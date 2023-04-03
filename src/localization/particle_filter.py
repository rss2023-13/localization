#!/usr/bin/env python2

import rospy
import tf.transformations as tf
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

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
                                          lidar_callback,
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          odom_callback,
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          pose_initialization,
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
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

        def publish_average_point(self, particles, probs):
            new_x = np.average(particles[:,0], weights=probs)
            new_y = np.average(particles[:,1], weights=probs)

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
            odom_msg.pose.pose.point = point
            odom_msg.pose.pose.orientation = orientation

            self.odom_pub(odom_msg)

        def lidar_callback(self, lidar_data):

            probs = self.sensor_model.evaluate(self.particles, lidar_data.ranges)
            self.particles = np.random.choice(self.particles, size=particles.shape[0], probs/probs.sum())
            self.probs = probs
            
            # Publish the "average pose" of the particles
            # TODO: Experiment with the weighted average
            self.publish_average_point(self.particles, self.probs)

        def odom_callback(self, odom_data):

            # First get the new particles from the motion model
            linear = odom_data.twist.twist.linear
            angular = odom_data.twist.twist.angular

            dx = linear.x
            dy = linear.y
            dtheta_x = angular.x
            dtheta_y = angular.y
            dtheta = np.arctan2(dtheta_y, dtheta_x)

            self.particles = self.motion_model.evaluate(self.particles, np.array([dx, dy, dtheta]))
            
            # Get the "average" particle through a weighted average
            self.publish_average_point(self.particles, self.probs)


        
        def pose_initialization(self):

            

            pass


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
