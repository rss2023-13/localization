#!/usr/bin/env python
import numpy as np

"""
This is an optional submission for the lab5 written portion autograder. 
This is so you can check to make sure that your understanding is 
correct before you get a headstart on the coding. You will not get 
credit for submitting to this autograder. It's just for your testing. 
This pset will be manually graded
"""

Rinv =  np.array([[np.cos(-np.pi/6), -np.sin(-np.pi/6), 0], [np.sin(-np.pi/6), np.cos(-np.pi/6), 0], [0, 0, 1]])

pose_world = np.array([[.2],[.1],[np.pi/60]]) # delta
pose_robot_delta = list(np.matmul(Rinv2, pose_world).T[0])
print(pose_robot_delta)



Rpi3 = np.array([[np.cos(np.pi/3), -np.sin(np.pi/3), 0], [np.sin(np.pi/3), np.cos(np.pi/3), 0], [0, 0, 1]])
new_pose_robot = list((np.matmul(Rpi3, pose_robot_delta) + np.array([[3],[4],[np.pi/3]]).T)[0].T)
print(new_pose_robot)

def answer_to_1i():
    """
    Return your answer to 1i in a python list, [x, y, theta]
    """
    # = [ 0.22320508, -0.01339746,  0.05235988]
    return pose_robot_delta

def answer_to_1ii():
    """
    Return your answer to 1ii in a python list, [x, y, theta]
    """
    # = [3.123205080756888, 4.186602540378444, 1.0995574287564276]
    return new_pose_robot


#q2

zmax = 10
d = 7
eps = 0.1
sigma = .5
eta = 1

def p_hit(zk):
  if 0 <= zk <= zmax:
    print("here1")
    return eta/(np.sqrt(2*np.pi*sigma**2)) * np.exp(-(zk-d)**2/(2*sigma**2))
  else:
    return 0

def p_short(zk):
  if (0 <= zk <= d) and (d != 0):
    print("here2")
    return (2/d)*(1-zk/d)
  else:
    return 0

def p_max(zk):
  if (zmax - eps) <= zk <= zmax:
    print("here3")
    return 1/eps
  else:
    return 0
  
def p_rand(zk):
  if 0 <= zk <= zmax:
    print("here4")
    return 1/zmax
  else:
    return 0

a_hit = .74
a_short = .07
a_max = .07
a_rand = .12

def sensor_model(zk):
  return a_hit * p_hit(zk) + a_short * p_short(zk) + a_max * p_max(zk) + a_rand * p_rand(zk)

print(sensor_model(10))


def answer_to_2():
    """
    Return your answers to 2 in a python list for the values z=0,3,5,8,10
    Each value should be a float
    """
    return [sensor_model(0), sensor_model(3), sensor_model(5), sensor_model(8), sensor_model(10)]

    #[0.032, 0.023428571428578904, 0.017912354448417746, 0.09190663043951833, 0.7120000089923066]

