# -*- coding:utf-8 -*-
# Create by steve in 17-5-7 at 下午5:16


import scripts.ImuResultReader
import scripts.ImuPreprocess
import scripts.PcSavedReader

import matplotlib.pyplot as plt

import numpy as np

import os
if __name__ == '__main__':

    dir_name = "/home/steve/Data/FastUwbDemo/3/"
    # dir_name = "/home/steve/Data/IU/16/"
    # dir_name = "/home/steve/tmp/test/15/"
    for tt in os.listdir(dir_name):
        if 'ttsv' in tt:
            irr = scripts.PcSavedReader.PcSavedReader(dir_name + tt)

            # irr = scripts.ImuResultReader.ImuResultReader(dir_name)

    np.savetxt(dir_name + "sim_imu.csv", irr.data_with_time, delimiter=',')

    ip = scripts.ImuPreprocess.ImuPreprocess(dir_name + "sim_imu.csv")
    ip.computezupt()
    ip.findvertex()
    print(ip.zupt_result)

    np.savetxt(dir_name + "sim_pose.csv", ip.vertics, delimiter=',')
    np.savetxt(dir_name + "all_quat.csv", ip.vertex_quat, delimiter=',')
    np.savetxt(dir_name + "sim_zupt.csv", ip.zupt_result, delimiter=',')
    np.savetxt(dir_name + "vertex_time.csv", ip.vertics_time, delimiter=",")

    print(ip.vertics.shape, " - ", ip.vertex_quat.shape, " - ", ip.vertics_time.shape)

    plt.figure()
    plt.plot(ip.vertics_time, 'r')

    # ip.findcorner()
    # ip.computeconerfeature()
    plt.show()
