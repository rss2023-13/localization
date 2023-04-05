#!/usr/bin/env python
from __future__ import division

import numpy as np


zmax = 200.0
#d = 7
eps = 1
sigma = 8.0
eta = 1
eps = 1

def p_hit(zk, d):
  if 0 <= zk <= zmax:
    #print("here1", eta/(np.sqrt(2*np.pi*sigma**2)) * np.exp(-(zk-d)**2/(2*sigma**2)))
    return eta/(np.sqrt(2*np.pi*sigma**2)) * np.exp(-(zk-d)**2/(2*sigma**2))
  else:
    return 0

def p_short(zk, d):
  if (0 <= zk <= d) and (d != 0):
    #print("here2", (2./d)*(1.-zk/d))
    return (2./d)*(1.-(zk/d))
  else:
    return 0

def p_max(zk):
  if (zmax - eps) <= zk <= zmax:
    # print("here3")
    return 1/eps
  else:
    return 0
  
def p_rand(zk):
  if 0 <= zk <= zmax:
    # print("here4")
    return 1/zmax
  else:
    return 0

a_hit = .74
a_short = .07
a_max = .07
a_rand = .12

# def sensor_model(zk):
#   return a_hit * p_hit(zk) + a_short * p_short(zk) + a_max * p_max(zk) + a_rand * p_rand(zk)

# print(sensor_model(10))

def p_total_excluding_hit(zk, d):
    return a_short * p_short(zk, d) + a_max * p_max(zk) + a_rand * p_rand(zk)

# compute and normalize rows of p_hit
p_hit_table =  np.array([np.array([p_hit(zk, d) for zk in range(0, 201)]) for d in range(0, 201)])

p_short_table =  np.array([np.array([p_short(zk, d) for zk in range(0, 201)]) for d in range(0, 201)])
p_short_table = p_short_table.T
print(p_hit_table)
p_hit_sums = p_hit_table.sum(axis=0, keepdims=True) # row sums
print(p_hit_sums)
p_hit_table = p_hit_table / p_hit_sums # scaling
print(p_hit_table.sum(axis=1, keepdims=True))

# build the full table
p_total_excluding_hit_table = np.array([np.array([p_total_excluding_hit(zk, d) for zk in range(0, 201)]) for d in range(0, 201)])
p_total_table = a_hit * p_hit_table + p_total_excluding_hit_table
print(p_total_table)
# normalize columns of p_total to 1
table_col_sums = p_total_table.sum(axis=1, keepdims = True) # col sums
print("col sums before", table_col_sums.shape)
sensor_model_table = p_total_table / table_col_sums # scaling
print("col sums after", sensor_model_table.sum(axis=1, keepdims = True))

sensor_model_table = sensor_model_table.T
print(sensor_model_table)
print(sensor_model_table.shape)

# plot the surface for visualization
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter


fig = plt.figure()
ax = fig.gca(projection='3d')

# Make data.
X = np.arange(0, 201, 1)
Y = np.arange(0, 201, 1)
X, Y = np.meshgrid(X, Y)
Z = p_short_table

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                    linewidth=0, antialiased=False)

# Customize the z axis.
ax.set_zlim(0, .15)
ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()




        # # # There is a zk and d matrix for each particle. Look up the element self.sensor_model_table[d(i), zk(i)], multiply over all i (all beams) for each particle
        # # for particle_beams in pixel_scans:
        # #     likelihoods = np.array([self.sensor_model_table[particle_beams[i], pixel_ranges[i]] for i in range(self.num_beams_per_particle)])

        # #     total_likelihood = np.prod(likelihoods) ** self.squash_parameter

        # #     all_observation_likelihoods.append(total_likelihood)

        # all_observation_likelihoods = np.array([np.prod(self.sensor_model_table[particle_beam, pixel_ranges]) ** self.squash_parameter for particle_beam in pixel_scans])

