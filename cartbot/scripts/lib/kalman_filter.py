#!/usr/bin/env python
# coding:utf-8
#参照 https://www.hellocybernetics.tech/entry/2018/03/08/131445 https://satomacoto.blogspot.com/2011/06/python.html

import rospy
import numpy as np
import time
import math

class KalmanFilter:
    # サーカムフレックス(circumflex)またはハット記号(hat symbol)では_hをつける
    # wikipediaの誤差の記事のepsilonよりepを使った

    #以下の初期値は例である
    x = np.array([0, 0])
    p = np.array([[10, 0],[0, 10]])
    A = np.array([[1.0, 0],[0, 1.0]])
    B = np.array([5, 10]) #Bt ?
    Q = np.array([[20, 0],[0, 20]]) #
    R = np.array([[20, 0],[0, 20]]) #謎

    def __init__(self, x_0, p_0, A, B, Q, R, u):
        self.x = x_0
        self.p = p_0
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.u = u

    def update(self, coordinate):

        A = self.A
        B = self.B
        Q = self.Q
        R = self.R
        u = self.u
        x = self.x
        p = self.p
        """
        カルマンフィルター
        https://ja.wikipedia.org/wiki/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%83%BC
        確率ロボティクスのP38~P40参照、各値の詳細が載せられておりわかりやすい
        この場合、システムの時間遷移に関する線形モデルがAになる。ukはnp.dot(B,u)にあたる。また、今回はノイズを足していない

        xtは状態ベクトルであり、縦ベクトルで表される。utは同時刻の制御ベクトルであり、縦ベクトルである。(今回の場合、x1,tはx座標、x2,tはy座標になるっぽい)
             | x1,t |
             | x2,t |
        xt = | x3,t | 
             |  :   |
             |  :   |  
             | xn,t |

             | u1,t |
             | u2,t |
        ut = | u3,t |
             |  :   |
             |  :   |
             | um,t |
            
        AtとBtも行列であり、Atはn*nの正方行列である。Btはn*mの行列であり、mは制御ベクトルutの次元である。
        状態と制御をそれぞれ行列AtとBtで乗算して足すことで、この状態遷移関数は線形になる。カルマンフィルタはこのような線形性を想定している。これを非線形性に適応させたのが拡張カルマンフィルタだが、それはまた今度。
        
        信念:ベイズ統計学では、確率はあくまで何らかの主観的な根拠に基づいて計算されるものであり、計算された確率分布を「信念」、ある自称に対する確率を「信念の度合い」と呼ぶ。確率ロボティクスでは、信念とある場合は確率分布、あるいは確率密度関数のことを指していると考えて良い。

        """
        #予測
        x_h = np.dot(A, x) + np.dot(B, u) # 今の時刻の予測推定値
        # p_hの値がある特定の数に近づけば、おそらくそれは精度が高いということ
        # システムの時間遷移はA、その後にpを掛けてシステムの時間遷移Aの転置行列を与える。Qの周りにあるGは省略してる。要注意
        p_h = np.dot(np.dot(A,p),np.transpose(A)) + Q # 今の時刻の予測誤差行列
        #更新
        #np.linalg.invは逆行列を求める。p_h+Rとしてるが、本来はp_hは対角化しないといけない?(その場合Cは直交行列でなければならない)
        K = np.dot(p_h,np.linalg.inv(p_h + R))
        # muとepの更新
        self.x = x_h + np.dot(K,(coordinate - x_h))
        self.p = p_h - np.dot(K,p_h)

    def status(self):
        return self.x, self.p

    def prediction(self):
        A = self.A
        B = self.B
        u = self.u
        x = self.x
        x_h = np.dot(A, x) + np.dot(B, u)
        print(x_h)
        return x_h

    #使ってない、使え
    """
    def identification(self,candinate):
        if len(candidate) > 0:
            min_distance = distanceBetween2Points(x,candinate[0])
            decision_coordinate = candinate[0]
            for c in candinate:
                distance = distanceBetween2Points(x,c)
                if distance < min_distance:
                    min_distance = distance
                    decision_coordinate = c
            return decision_coordinate
    """