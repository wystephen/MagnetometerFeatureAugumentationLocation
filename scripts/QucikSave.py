# -*- coding:utf-8 -*-
# Create by steve in 17-5-7 at 下午5:16


import scripts.ImuResultReader

import numpy as np

if __name__ == '__main__':
    irr = scripts.ImuResultReader.ImuResultReader("/home/steve/Data/10DOFIMU/Record(6).txt")
    np.savetxt("/home/steve/Data/FastUwbDemo/1/sim_imu.csv", irr.data_with_time, "%.18e", delimiter=',')
