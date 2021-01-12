#!/usr/bin/env python
# coding:utf-8

import rospy
import matplotlib.pyplot as plt
import numpy as np
from lib.kalman_filter import KalmanFilter

if __name__ == '__main__':
    # データ作成
    T = 20 #データ数
    x_0 = np.array([0, 0])#実データは0,0から始まる
    p_0 = np.array([[1, 0],
                   [0, 1]]) #謎

    #解釈: https://qiita.com/IshitaTakeshi/items/740ac7e9b549eee4cc04 基準
    A = np.array([[1.001, 0.001],
                  [0, 0.99]]) #Ft ?
    b = np.array([0.5, 1]) #Bt ?
    Q = np.array([[1, 0],
                  [0, 1]]) #
    R = np.array([[1, 0],
                  [0, 1]]) #謎
    u = np.array([[1,0],[0,1]])

    rvq = np.random.multivariate_normal(np.zeros(2), Q, T)
    rvr = np.random.multivariate_normal(np.zeros(2), R, T)
    obs = np.zeros((T, 2))
    obs[0] = x_0
    #obsは実データ
    for i in range(1, T):
        obs[i] = np.dot(A,obs[i-1]) + b + rvq[i] + rvr[i]

    x_list = np.array([x_0])
    p_list = np.array([p_0])
    print(x_list)
    kf = KalmanFilter(x_0, p_0, A, b, Q, R, u)
    for o in obs:
        kf.update(o)
        x, p = kf.status()
        x_list = np.vstack((x_list, x))

    fig = plt.figure(figsize=(16, 9))
    ax = fig.gca()

    ax.scatter(obs[:, 0], obs[:, 1], s=10, alpha=1,
               marker="o", color='w', edgecolor='k')
    ax.plot(obs[:, 0], obs[:, 1], alpha=0.5, lw=1, color='k')

    print(x_list)
    print(p_list)

    ax.scatter(x_list[:, 0], x_list[:, 1],
               s=10, alpha=1, marker="o", color='r')
    ax.plot(x_list[:, 0], x_list[:, 1], alpha=0.5, lw=1, color='r')

    plt.show()
