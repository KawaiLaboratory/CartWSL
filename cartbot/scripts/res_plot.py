#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pcl
import numpy as np
import scipy as sp
from scipy.optimize.slsqp import approx_jacobian
import matplotlib.pyplot as plt
import rospy
import os
import csv


#self made
from lib.time_watcher import TimeWatcher as tw

if __name__ == '__main__':

    rospy.init_node('res_plot', anonymous=True)

    """
    #pcdファイルから直接値を得ようとした時の残骸
    count = 0
    while(os.path.exists(os.environ.get("CATKIN_FOLDER")+"/observation_data/past/o_lrf_"+str(count)+".pcd")):
        Points = []
        P = pcl.load(os.environ.get("CATKIN_FOLDER")+"/observation_data/past/o_lrf_"+str(count)+".pcd")
        for p in P:
            Points.append([p[0],p[1]])
    """

    feature_list_0 = []
    feature_list_1 = []
    feature_list_2 = []

    with open(os.environ.get("CATKIN_FOLDER")+'/leg_features_data.csv', 'r') as f:
        reader = csv.reader(f)
        for r in reader:
            feature_list_0.append(r[0])
            feature_list_1.append(r[1])
            feature_list_2.append(r[2])

    # Figureの初期化
    fig = plt.figure(figsize=(65, 10)) #...1
    # Figure内にAxesを追加()
    ax0 = fig.add_subplot(131) #...2
    ax0.scatter(feature_list_0, feature_list_1, label="test") #...3
    ax0.set_xlabel('grith')
    ax0.set_ylabel('width')

    ax1 = fig.add_subplot(132) #...2
    ax1.scatter(feature_list_0, feature_list_2, label="test", c="red") #...3
    ax1.set_xlabel('grith')
    ax1.set_ylabel('depth')

    ax2 = fig.add_subplot(133) #...2
    ax2.scatter(feature_list_1, feature_list_2, label="test", c="green") #...3
    ax2.set_ylabel('width')
    ax2.set_xlabel('depth')

    # 凡例の表示
    plt.legend()

    # プロット表示(設定の反映)
    plt.show()
