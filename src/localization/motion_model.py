import math
import numpy as np
# import rospy

class MotionModel:

    def __init__(self):

        # self.deterministic = rospy.get_param("~deterministic", False)
        self.deterministic = False

    def rotate_vectorized(self, odometry, thetas):
        """
        thetas is an array of angles corresponding to the pose of the original particles
        """

        N = thetas.shape[0]

        dx_copies = np.ones_like(thetas) * odometry[0]
        dy_copies = np.ones_like(thetas) * odometry[1]
        dtheta_copies = np.ones_like(thetas) * odometry[2]

        odometry_n_copies = np.array([dx_copies, dy_copies, dtheta_copies]).T

        if self.deterministic:
            x_scale_factor = 0
            y_scale_factor = 0
            theta_scale_factor = 0
        else:
            #simulation: 2, .02, 1
            #robot: 2, .1, 3
            x_scale_factor = 2 
            y_scale_factor = 0.02
            theta_scale_factor = 1

        max_x_scale = x_scale_factor * np.abs(odometry[0])
        max_theta_scale = theta_scale_factor * np.abs(odometry[2])

        odometry_n_copies[:,0] = odometry_n_copies[:,0] + np.random.normal(scale=max_x_scale, size=N)
        odometry_n_copies[:,1] = odometry_n_copies[:,1] + np.random.normal(scale=y_scale_factor, size=N)
        odometry_n_copies[:,2] = odometry_n_copies[:,2] + np.random.normal(scale=max_theta_scale, size=N)

        cosines = np.cos(thetas)
        sines = np.sin(thetas)
        zeros = np.zeros_like(thetas)
        ones = np.ones_like(thetas)
        matrices = np.array([[cosines, -sines, zeros],
                             [sines, cosines, zeros],
                             [zeros, zeros, ones]])

        matrices = np.transpose(matrices, axes=(2, 0, 1))
        rotated = np.transpose(np.matmul(matrices, odometry_n_copies.T), axes=(0,2,1)) #(N, N, 3)

        return rotated[np.arange(N), np.arange(N), :]

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

        return new_particles
        
if __name__ == "__main__":
    test_motion = MotionModel()
    test_particles = np.array([[3,4,np.pi/3],
                               [7,6,np.pi/4],
                               [3.5, 2, np.pi/3.5]])
    # part_2 = np.array([[7,6,np.pi/4]])
    test_odom = np.array([0.2232, -0.013397, np.pi/60])
    new_particles = test_motion.evaluate(test_particles, test_odom)
    print(new_particles)
