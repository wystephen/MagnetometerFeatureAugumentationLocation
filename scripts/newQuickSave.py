# -*- coding:utf-8 -*-
# Create by steve in 17-5-19 at 上午9:55



import ImuResultReader
import ImuPreprocess
import PcSavedReader

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import numpy as np

import os

if __name__ == '__main__':
    dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/46/"
    # dir_name = "/home/steve/Code/Mini-IMU/Scripts/IMUWB/20/"
    # dir_name = "/home/steve/Data/FastUwbDemo/2/"
    # dir_name = "/home/steve/tmp/test/20/"


    a = np.loadtxt(dir_name + 'imu.txt', delimiter=',')
    print(a[:, 1:4].shape)
    # a[:,1:4] *= 9.816

    np.savetxt(dir_name + "sim_imu.csv", a, delimiter=',')

    ip = ImuPreprocess.ImuPreprocess(dir_name + "sim_imu.csv")
    ip.computezupt()
    # plt.show()
    ip.findvertex()
    print(ip.zupt_result)

    np.savetxt(dir_name + "sim_pose.csv", ip.vertics, delimiter=',')
    np.savetxt(dir_name + "all_quat.csv", ip.vertex_quat, delimiter=',')
    np.savetxt(dir_name + "sim_zupt.csv", ip.zupt_result, delimiter=',')
    np.savetxt(dir_name + "vertex_time.csv", ip.vertics_time, delimiter=",")
    np.savetxt(dir_name + "vertex_high.csv", ip.vertics_high, delimiter=',')

    print(ip.vertics.shape, " - ", ip.vertex_quat.shape, " - ", ip.vertics_time.shape)

    trace_fig = plt.figure()
    ax = trace_fig.gca(projection='3d')
    ax.plot(ip.vertics[:, 0], ip.vertics[:, 1], ip.vertics[:, 2], 'r*-')
    ax.legend()

    # plt.figure()
    # plt.plot(ip.vertics_time, 'r')

    # ip.findcorner()
    # ip.computeconerfeature()
    plt.show()
