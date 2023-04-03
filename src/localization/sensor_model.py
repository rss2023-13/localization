import numpy as np
from localization.scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D` 
# if any error re: scan_simulator_2d occurs

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.lidar_scale_to_map = 1

        ####################################
        # TODO
        # Adjust these parameters
        self.a_hit = .74
        self.a_short = .07
        self.a_max = .07
        self.a_rand = .12
        self.sigma = 8

        self.eta = 1 # 

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201

        self.zmax = 10 #10 meter max lidar scan range

        self.zmax_pixel = 200 # since table has index values 0 through 200

        self.map_resolution = 10 / 201# meters / pixel, this determines how we bucket the lidar

        self.squash_parameter = 1 / 2.2
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)
        


    def p_hit(self, zk, d):
        if 0 <= zk <= self.zmax:
            return self.eta/(np.sqrt(2*np.pi*self.sigma**2)) * np.exp(-(zk-self.d)**2/(2*self.sigma**2))
        else:
            return 0

    def p_short(self, zk, d):
        if (0 <= zk <= self.d) and (self.d != 0):
            return (2/self.d)*(1-zk/self.d)
        else:
            return 0

    def p_max(self, zk):
        if zk == self.zmax:
            return 1
        else:
            return 0
        
    def p_rand(self, zk):
        if 0 <= zk <= self.zmax:
            return 1/self.zmax
        else:
            return 0


    # def p_total(self, zk, d): # dont need this
    #     return self.a_hit * self.p_hit(zk, d) + self.a_short * self.p_short(zk, d) + self.a_max * self.p_max(zk) + self.a_rand * self.p_rand(zk)

    def p_total_excluding_hit(self, zk, d):
        return self.a_short * self.p_short(zk, d) + self.a_max * self.p_max(zk) + self.a_rand * self.p_rand(zk)



    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        # compute and normalize rows of p_hit
        p_hit_table = np.array([np.array([self.p_hit(zk, d) for zk in range(0, self.table_width)]) for d in range(0, self.table_width)])
        p_hit_sums = p_hit_table.sum(axis=0, keepdims=True) # row sums
        p_hit_table = p_hit_table / p_hit_sums # scaling

        # build the full table
        p_total_excluding_hit_table = np.array([np.array([self.p_total_excluding_hit(zk, d) for zk in range(0, self.table_width)]) for d in range(0, self.table_width)])
        p_total_table = self.a_hit * p_hit_table + p_total_excluding_hit_table

        # normalize columns of p_total to 1
        table_col_sums = p_total_table.sum(axis=1, keepdims = True) # col sums
        
        self.sensor_model_table = p_total_table / table_col_sums # scaling

        print(self.sensor_model_table.shape)
        # # plot the surface for visualization
        # from mpl_toolkits.mplot3d import Axes3D
        # import matplotlib.pyplot as plt
        # from matplotlib import cm
        # from matplotlib.ticker import LinearLocator, FormatStrFormatter


        # fig = plt.figure()
        # ax = fig.gca(projection='3d')

        # # Make data.
        # X = np.arange(0, self.table_width, 1)
        # Y = np.arange(0, self.table_width, 1)
        # X, Y = np.meshgrid(X, Y)
        # Z = self.sensor_model_table

        # # Plot the surface.
        # surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
        #                     linewidth=0, antialiased=False)

        # # Customize the z axis.
        # ax.set_zlim(0, .15)
        # ax.zaxis.set_major_locator(LinearLocator(10))
        # ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

        # # Add a color bar which maps values to colors.
        # fig.colorbar(surf, shrink=0.5, aspect=5)

        # plt.show()


    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. => zk(i) beams

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """
        #pre check
        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.

        # This produces a matrix of size N x num_beams_per_particle 

        #scans is d - actual distance (ray casting)
        # these
        scans = self.scan_sim.scan(particles)

        #ranges are zk, measured distance..
        ranges = np.array(observation.ranges)

        # Downsample the lidar beams to num_beams_per_particle
        total_num_ranges = len(ranges)

        downsample_indices = np.round(np.linspace(0, total_num_ranges, self.num_beams_per_particle)).astype(int)

        downsampled_ranges = ranges[downsample_indices]

        #clip above zmax and below 0 #TODO
        max_clipped_ranges = np.where(downsampled_ranges > self.zmax, self.zmax, downsampled_ranges)

        final_ranges = np.where(max_clipped_ranges < 0, 0, max_clipped_ranges) # clip below zero

        max_clipped_scans = np.where(scans > self.zmax, self.zmax, scans)

        final_scans = np.where(max_clipped_scans < 0, 0, max_clipped_scans) # clip below zero

        
        

        #converting meters to pixels for observations and ground truth scans
        pixel_ranges = round(final_ranges / (self.map_resolution * self.lidar_scale_to_map)) # round this to an int

        pixel_scans = round(final_scans / (self.map_resolution * self.lidar_scale_to_map))

        
        # There is a zk and d matrix for each particle. Look up the element self.sensor_model_table[zk(i), d(i)], multiply over all i (all beams) for each particle
        





        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
