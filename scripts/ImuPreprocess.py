# -*- coding:utf-8 -*-
# Create by steve in 17-4-14 at 上午11:03


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

# from OPENSHOE import zupt_test
from scripts.OPENSHOE import Setting, zupt_test


class ImuPreprocess:
    def __init__(self, source_data_file):
        '''
        Load data from file.
        :param source_data_file: 
        '''
        self.data = np.loadtxt(source_data_file, dtype=float, delimiter=',')
        # use rad replace deg
        self.data[:, 4:7] = self.data[:, 4:7] * np.pi / 180.0
        self.data[:, 1:4] = self.data[:, 1:4] * 9.8

        self.para = Setting.settings()
        # self.para.time_Window_size = 40

    def computezupt(self):
        '''
        
        :return: 
        '''
        zupt_detector = zupt_test.zupte_test(self.para)
        self.zupt_result = zupt_detector.GLRT_Detector(self.data[:, 1:7])

        # debug

        plt.figure()
        plt.title("ZUPT detector")
        plt.grid(True)
        for i in range(1, 4):
            plt.plot(self.data[:, i])
        plt.plot(self.zupt_result)
        plt.show()

    def ShowMagnety(self):
        plt.figure()



if __name__ == '__main__':
    ip = ImuPreprocess(source_data_file="../TMP_DATA/all_data.csv")
    ip.computezupt()
