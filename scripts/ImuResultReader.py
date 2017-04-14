

import matplotlib.pyplot as plt

import numpy  as np
import scipy as sp

import time,datetime


import re

class ImuResultReader:
    def __init__(self,file_name):
        '''
        
        :param file_name: 
        '''
        self.file_handle = open(file_name)

        # Read whole file
        self.line_list = self.file_handle.readlines()

        # Read first line set initial time stamp
        self.start_time = time.mktime(time.strptime(self.line_list[0],u"﻿开始时间：%Y年%m月%d日 %H:%M:%S "))

        print("start time :",self.start_time)
        # Read second line insure data type
        self.data_category_num = len(self.line_list[1].split('：'))-2
        print("DOF of data:",self.data_category_num,'\n',self.line_list[1])

        # Initial data(float)
        self.data_with_time = np.zeros([len(self.line_list)-2,self.data_category_num+1])


        # Initial re ...
        pattern = re.compile('\\s+[-]*\\d+[\\.]+\\d+')

        #Loop read data
        for i in range(self.data_with_time.shape[0]):
            index = i +2
            try:

                self.data_with_time[i,0] = time.mktime(time.strptime(self.line_list[index].split('     ')[0].split('.')[0],"%Y-%m-%d %H:%M:%S"))
                self.data_with_time[i,0] += float(self.line_list[index].split('     ')[0].split('.')[1])/1000.0
            except ValueError:
                print("some Value Error happend")
                continue
            # print(float(self.line_list[index].split('     ')[0].split('.')[1])/1000.0)

            # First line of raw data(1.set the time stamp as start time point)
            if i is 0:
                self.time_offset = self.start_time - self.data_with_time[i,0]
            self.data_with_time[i,0] += self.time_offset
            # print('{0}'.format(self.data_with_time[i,0]))
            # print(pattern.findall(self.line_list[index]))
            # print(len(pattern.findall(self.line_list[index])))
            tt = pattern.findall(self.line_list[index])
            # tt = list(tt)
            if len(pattern.findall(self.line_list[index]))> 9:

                for j in range(1,self.data_with_time.shape[1]):
                    self.data_with_time[i,j] = float(tt[j-1])



            # tmp_list = self.line_list[index].split('     ')[1:]
            # for j in range(1,self.data_with_time.shape[1]):
            #     if not j ==  8:
            #         self.data_with_time[i,j]=float(tmp_list[j-1])
            #     else:
            #
            #
            #         self.data_with_time[i,j] = float(tmp_list[j-1].split(' ')[0])
            #         self.data_with_time[i,j+1] = float(tmp_list[j-1].split(' ')[1])
            #         self.data_with_time[i,j+2] = float(tmp_list[j])

    def SavetoFile(self, file_name):
        '''
        save time accx accy accz wx wy wz mx my mz pressure height
        :param file_name: 
        :return: 
        '''
        data_save = np.zeros([self.data_with_time.shape[0], 12])
        data_save[:, :7] = self.data_with_time[:, :7]
        print(data_save[10, :])
        data_save[:, 7:] = self.data_with_time[:, -5:]
        np.savetxt(file_name, data_save, delimiter=",")


if __name__ == '__main__':
    irr = ImuResultReader("/home/steve/Data/10DOFIMU/Record(2).txt")
    # np.savetxt("../TMP_DATA/all_data.csv",irr.data_with_time,"%.18e",
    #            ',')
    irr.SavetoFile("../TMP_DATA/all_data.csv")

    plt.figure(1)
    plt.grid(True)

    for i in range(1,9):
        plt.plot(irr.data_with_time[:,i])

    plt.show()

