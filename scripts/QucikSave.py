# -*- coding:utf-8 -*-
# Create by steve in 17-5-7 at 下午5:16


import scripts.ImuResultReader
import scripts.ImuPreprocess

import matplotlib.pyplot as plt

import numpy as np

if __name__ == '__main__':
    irr = scripts.ImuResultReader.ImuResultReader("/home/steve/Data/10DOFIMU/Record(7).txt")
    np.savetxt("/home/steve/Data/FastUwbDemo/2/sim_imu.csv", irr.data_with_time, delimiter=',')

    ip = scripts.ImuPreprocess.ImuPreprocess("/home/steve/Data/FastUwbDemo/2/sim_imu.csv")
    ip.computezupt()
    ip.findvertex()
    print(ip.zupt_result)

    np.savetxt("/home/steve/Data/FastUwbDemo/2/sim_pose.csv", ip.vertics, delimiter=',')
    np.savetxt("/home/steve/Data/FastUwbDemo/2/all_quat.csv", ip.vertex_quat, delimiter=',')
    np.savetxt("/home/steve/Data/FastUwbDemo/2/sim_zupt.csv", ip.zupt_result, delimiter=',')
    np.savetxt("/home/steve/Data/FastUwbDemo/2/vertex_time.csv", ip.vertics_time, delimiter=",")

    print(ip.vertics.shape, " - ", ip.vertex_quat.shape, " - ", ip.vertics_time.shape)

    plt.figure()
    plt.plot(ip.vertics_time, 'r')

    # ip.findcorner()
    # ip.computeconerfeature()
    plt.show()
