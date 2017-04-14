# -*- coding:utf-8 -*-
# Create by steve in 17-4-14 at 上午11:03


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from OPENSHOE import zupt_test
from OPENSHOE import Setting


class ImuPreprocess:
    def __init__(self, source_data_file):
        '''
        Load data from file.
        :param source_data_file: 
        '''
        self.data = np.loadtxt(source_data_file, dtype=float, delimiter=',')
        self.para = Setting.setting()

    def computezupt(self):
        '''
        
        :return: 
        '''
        zupt_detector = zupte_test()
        self.zupt_result = zupt_detector.GLRT_Detector(self.data[:, 1:7])

        # debug

        plt.figure(1)
        plt.grid(True)
        plt.plot(self.zupt_result)
        plt.show()


if __name__ == '__main__':
    ip = ImuPreprocess(source_data_file="../TMP_DATA/all_data.csv")
    ip.computezupt()
