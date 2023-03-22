import math
import numpy as np

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        pass

        ####################################

    def rotate(self, odometry, theta):
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0], 
                            [math.sin(theta), math.cos(theta), 0],
                            [0, 0, 1]])

        return np.matmul(rotation_matrix, odometry)

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        new_particles = np.zeros_like(particles)
        for i, particle in enumerate(particles):
            new_particles[i] = particle + self.rotate(odometry, particle[2])

        return new_particles
        
        
