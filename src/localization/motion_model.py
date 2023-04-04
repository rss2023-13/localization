import math
import numpy as np
import rospy

class MotionModel:

    DETERMINISTIC = rospy.get_param("~deterministic", True)

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.
	    pass
        ####################################

    def rotate_vectorized(self, odometry, thetas):
        """
        thetas is an array of angles corresponding to the pose of the original particles
        """

        cosines = np.cos(thetas)
        sines = np.sin(thetas)
        zeros = np.zeros_like(thetas)
        ones = np.ones_like(thetas)
        matrices = np.array([[cosines, -sines, zeros],
                             [sines, cosines, zeros],
                             [zeros, zeros, ones]])

        matrices = np.transpose(matrices, axes=(2,0,1))
        return np.matmul(matrices, odometry)

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
        N = particles.shape[0]
        world_odom = self.rotate_vectorized(odometry, particles[:, 2])
        new_particles = particles + world_odom

        # Adding noise - higher variance if the odometry data is high magnitude
        # if self.DETERMINISTIC:
	    #     scale_factor = 0
	    # else: scale_factor = 0.1

        scale_factor = 0.1

        max_x_scale = scale_factor * np.abs(odometry[0])
        max_y_scale = scale_factor * np.abs(odometry[1])
        max_theta_scale = scale_factor * np.abs(odometry[2])

        new_particles[:,0] = new_particles[:,0] + np.random.normal(scale=max_x_scale, size=N)
        new_particles[:,1] = new_particles[:,1] + np.random.normal(scale=max_y_scale, size=N)
        new_particles[:,2] = new_particles[:,2] + np.random.normal(scale=max_theta_scale, size=N)

        return new_particles
        
if __name__ == "__main__":
    test_motion = MotionModel()
    test_particles = np.array([[3,4,np.pi/3],
                               [7,6,np.pi/4]])
    # part_2 = np.array([[7,6,np.pi/4]])
    test_odom = np.array([0.2232, -0.013397, np.pi/60])
    new_particles = test_motion.evaluate(test_particles, test_odom)
    print(new_particles)
