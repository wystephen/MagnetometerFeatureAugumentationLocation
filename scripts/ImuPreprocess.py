# -*- coding:utf-8 -*-
# Create by steve in 17-4-14 at 上午11:03


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

# from OPENSHOE import zupt_test
from scripts.OPENSHOE import Setting, zupt_test, ZUPT


class ImuPreprocess:
    def __init__(self, source_data_file):
        '''
        Load data from file.
        :param source_data_file: 
        '''
        self.data = np.loadtxt(source_data_file, dtype=float, delimiter=',')
        # use rad replace deg
        self.data[:, 4:7] = self.data[:, 4:7] * np.pi / 180.0
        # use m/s^2 replace g.
        self.data[:, 1:4] = self.data[:, 1:4] * 9.8

        self.para = Setting.settings()
        self.para.sigma_a *= 6.0
        self.para.sigma_g *= 6.0

        self.para.sigma_acc *= 6.0
        self.para.sigma_gyro *= 6.0

        self.para.Ts = np.mean(self.data[1:, 0] - self.data[:-1, 0])
        print("TS :", self.para.Ts)
        # self.para.time_Window_size = 5

    def computezupt(self):
        '''
        
        :return: 
        '''
        zero_velocity_detector = zupt_test.zv_detector(self.para)
        self.zupt_result = zero_velocity_detector.GLRT_Detector(self.data[:, 1:7])

        # debug

        plt.figure()
        plt.title("ZUPT detector")
        plt.grid(True)
        for i in range(1, 4):
            plt.plot(self.data[:, i])
        plt.plot(self.zupt_result * 80.0)
        # plt.show()

        # ZUPT
        ins_filter = ZUPT.ZUPTaidedInsPlus(self.para)

        ins_filter.init_Nav_eq(self.data[:30, 1:7])

        self.trace_x = np.zeros([9, self.data.shape[0]])

        for index in range(self.data.shape[0]):
            # if(index > 2):
            #     self.para.Ts = self.data[index,0]-self.data[index-1,0]
            self.trace_x[:, index] = ins_filter.GetPosition(self.data[index, 1:7], self.zupt_result[index]).reshape(18)[
                                     :9]
        self.trace_x = self.trace_x.transpose()

        print(self.trace_x.shape)

        plt.figure()
        plt.title("filter result ")
        plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], '.-')
        plt.grid(True)








    def ShowMagnety(self):
        plt.figure()
        plt.subplot(4, 1, 1)
        plt.title("norm")
        # print(np.linalg.norm(self.data[:,7:10],axis=-1).sh)
        plt.plot(np.linalg.norm(self.data[:, 7:10], axis=-1) ** 0.5)

        plt.subplot(4, 1, 2)
        plt.title("all")
        for i in range(3):
            plt.plot(self.data[:, i + 7], label=str(i))
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.title(" pressure")
        plt.plot(self.data[:, 10] - np.mean(self.data[:, 10]))
        print(min(self.data[:, 11]), max(self.data[:, 11]))





if __name__ == '__main__':
    ip = ImuPreprocess(source_data_file="../TMP_DATA/all_data.csv")

    ip.computezupt()

    ip.ShowMagnety()

    plt.show()
