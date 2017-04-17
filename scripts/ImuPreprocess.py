# -*- coding:utf-8 -*-
# Create by steve in 17-4-14 at 上午11:03


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

# from OPENSHOE import zupt_test
from scripts.OPENSHOE import Setting, zupt_test, ZUPT

import array

import os

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

        return

    def computezupt(self):
        '''
        run zero velocity detector and zupt location.
         (If tmp_data/trace.txt and tmp_data/zupt_result.txt exists,
         Just load data from file to save time.)
        :return: 
        '''

        if "tmp_data" in os.listdir("./"):
            # print(os.listdir("./"))
            if "zupt_result.txt" in os.listdir("./tmp_data") and "trace.txt" in os.listdir("./tmp_data"):

                self.trace_x = np.loadtxt("./tmp_data/trace.txt")
                self.zupt_result = np.loadtxt("./tmp_data/zupt_result.txt")

                if self.trace_x.shape[0] == self.data.shape[0]:
                    plt.figure()
                    plt.title("ZUPT detector(quick load)")
                    plt.grid(True)
                    for i in range(1, 4):
                        plt.plot(self.data[:, i])
                    plt.plot(self.zupt_result * 80.0)

                    plt.figure()
                    plt.title("filter result (quick load)")
                    plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], '.-')
                    plt.grid(True)

                    return

        else:
            os.mkdir("tmp_data")

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
        np.savetxt("./tmp_data/trace.txt", self.trace_x)
        np.savetxt("./tmp_data/zupt_result.txt", self.zupt_result)

        plt.figure()
        plt.title("filter result ")
        plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], '.-')
        plt.grid(True)

        return



    def ShowMagnety(self):
        '''
        Find and show simple feature in sequence.
        :return: 
        '''
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
        return

    def findvertex(self):
        '''
        Find vertex and compute edge(in trace graph not optimize graph).
        :return: 
        '''

        vertex_point = array.array('f')
        data_cols = 9

        for i in range(1, self.trace_x.shape[0]):
            if self.zupt_result[i] > 0.5 and self.zupt_result[i - 1] < 0.5:

                for j in range(data_cols):
                    vertex_point.append(self.trace_x[i, j])
                    # vertex_point.append(self.trace_x[i,0])
                    # vertex_point.append(self.trace_x[i,1])
                    # vertex_point.append(self.trace_x[i,2])
                    # vertex_point
        self.vertics = np.frombuffer(vertex_point, dtype=np.float32)
        self.vertics = np.reshape(self.vertics, (-1, data_cols))
        print(self.vertics[20, :])

        plt.figure()
        plt.title("show vertices in trace")
        plt.grid(True)

        plt.plot(self.trace_x[:, 0], self.trace_x[:, 1], 'r.-', label="trace")
        plt.plot(self.vertics[:, 0], self.vertics[:, 1], 'bo', label="vertex")

        plt.legend()

        return

    def findcorner(self):
        '''
        Find corner in vertex point.
        :return: 
        '''

        plt.figure()
        plt.title("show x y z (in findcorner)")

        for i in range(3):
            plt.plot(self.vertics[:, i], '-+', label=str(i))
        plt.grid(True)
        plt.legend()

        corner = array.array('f')

        # plt.figure()
        # plt.title("\\delta y / \\delta x")
        tmp = (self.vertics[1:, 1] - self.vertics[:-1, 1]) / (self.vertics[1:, 0] - self.vertics[:-1, 1])
        plt.plot(tmp, label="\deltay / \delta x")
        plt.legend()
        plt.grid(True)

        threshold = 0.2
        length = 5

        itcounter = 0

        for i in range(self.vertics.shape[0]):
            # if itcounter > 0:
            #     itcounter -= 1
            #     continue

            if i == 1 or i == self.vertics.shape[0] - 1:
                for j in range(self.vertics.shape[1]):
                    corner.append(self.vertics[i, j])
            elif i < length + 1 or i > self.vertics.shape[0] - length - 1:
                itcounter = 0
                continue
            else:
                # if (np.abs(self.vertics[i, 0] -
                #                    (self.vertics[i - 1, 0] + self.vertics[i + 1, 0]) / 2.0) > threshold or
                #             np.abs(self.vertics[i, 1] -
                #                            (self.vertics[i - 1, 1] + self.vertics[i + 1, 1]) / 2.0) > threshold):
                if ((self.vertics[i, 0] - self.vertics[i - length, 0]) * (
                    self.vertics[i + length, 0] - self.vertics[i, 0]) < 0.0 and
                        (abs((self.vertics[i, 0] - self.vertics[i - length, 0]) + (
                            self.vertics[i + length, 0] - self.vertics[i, 0])) > threshold
                         ) or (
                                    (self.vertics[i, 1] - self.vertics[i - length, 1]) * (
                                    self.vertics[i + length, 1] - self.vertics[i, 1]) < 0.0 and
                                abs((self.vertics[i, 1] - self.vertics[i - length, 1]) + (
                                    self.vertics[i + length, 1] - self.vertics[i, 1])) > threshold)
                    ):
                    itcounter += 1
                    # if itcounter < 3 or itcounter > 4:
                    #     continue

                    for j in range(self.vertics.shape[1]):
                        corner.append(self.vertics[i, j])
                        # itcounter += length-3
                else:
                    itcounter = 0



        self.corner = np.frombuffer(corner, dtype=np.float32).reshape([-1, self.vertics.shape[1]])

        # for i in range(3):
        #     plt.plot(self.corner[:,i],'ro')

        plt.figure()
        plt.title("corner in vertex")
        plt.plot(self.vertics[:, 0], self.vertics[:, 1], 'r-+', label="vertics")
        plt.plot(self.corner[:, 0], self.corner[:, 1], 'bo', label="corner")

        plt.legend()
        plt.grid(True)


if __name__ == '__main__':
    ip = ImuPreprocess(source_data_file="../TMP_DATA/all_data.csv")

    ip.computezupt()

    ip.ShowMagnety()

    ip.findvertex()

    ip.findcorner()

    plt.show()
