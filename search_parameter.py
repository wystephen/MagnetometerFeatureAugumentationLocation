#!/usr/bin/env python
# coding=utf-8
import os
import numpy


for fi in [1,2,10,20,50,80,100,300,800,1000,1300,1600,1900,2300,2600,4000,4500,5000,7000,10000,15000]:
    for si in [1,2,10,20,50,80,100,300,800,1000,1300,1600,1900,2300,2600,4000,4500,5000,7000,10000,15000]:
        for di in [0.00001,0.00002,0.00004,0.0001,0.0003,0.0007,0.001,0.003,0.007,0.01,0.05,0.1,0.5,1.0,1.5,3.0]:
            os.system("./cmake-build-debug/graph_build_without_ekf {0} {1} {2}".format(
                fi,si,di
                ))
            print("finished {0} {1} {2}".format(
                fi,si,di
                ))

