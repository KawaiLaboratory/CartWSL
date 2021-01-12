#!/usr/bin/env python
# coding:utf-8

import re
import csv
import time
import os
import math

#学習用
import sklearn
import numpy as np
from sklearn import svm, datasets
import matplotlib.pyplot as plt
from itertools import product

#ROS
import rospy
from std_msgs.msg import String #いる?
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
from std_msgs.msg import String

#self made
from lib.kalman_filter import KalmanFilter
import lib.extended_math as em
from lib.time_watcher import TimeWatcher

#配信用変数 --ここから--
pub = rospy.Publisher('/center_point', PointCloud2, queue_size=1)
HEADER = Header(frame_id='/laser')
FIELDS = [
    # 点の座標(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    # 点の色(RGB)
    # 赤: 0xff0000, 緑:0x00ff00, 青: 0x0000ff
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]
POINTS = []#x,y,z,RGB
def pub_points():
    print(" ")
#確認用 基本使わない
def pub_confirmation():
    x=0.1
    y=0.5
    z=0
    POINTS.append([x,y,z,0x00ff00])
    while not rospy.is_shutdown():
        pub_points()
# --ここまで--

"""
def dt_update():
    if t <= t_swing:
        d_t = -(d_max - d_stance)/2*math.cos(2*math.pi/t_swing*t)+(d_max+d_stance)/2
    elif t>= t_swing and t <= t_cycle:
        d_t = d_stance

    t_cycle = t_swing+t_stance
"""

def callback(data):

    global former_prediction_point
    global xv,yv
    global prediction_range
    global former_T
    global kf

    swb1 = False

    lap_T = tw.lap()
    T = former_T + lap_T

    kf.B = np.array([xv*T*1.2,yv*T*1.2])
    kf.update([former_prediction_point[0],former_prediction_point[1]])

    #中心点の配信用
    POINTS = []

    #下にあるreceive_str_listをfloatにしたもの
    receive_float_list = []
    #受信したデータを;で分けたもの
    receive_str_list = data.data.split(";")
    #float化
    for rsl in receive_str_list:
        if rsl != "":
            receive_float_list.append([float(rs) for rs in rsl.split(",")])

    #number_list.data[0]が0の時、検知できたことを表し、1の時、検知失敗したことを表す。
    number_list = Float32MultiArray()

    #受信データがある場合とない場合
    #ある場合
    if len(receive_float_list) > 0:
        receive_float_list = np.array(receive_float_list)
        #受信値の特徴データ
        receive_feature_list = receive_float_list[:,:feature_number]
        #受信値の中心座標データ
        receive_center_list = receive_float_list[:,feature_number:]
        #人間検知
        result = clf_a.predict(receive_feature_list)
        #応急処置、中心点を始めに検知した方に向かう
        min_difference = 50
        kf_status_x,kf_status_p = kf.status()
        POINTS.append([kf_status_x[0],kf_status_x[1],0,0xffff00])

        #足として検知したものの中心点をリスト化
        #x,y,z,svmの結果
        detected_legs = filter(lambda a: a[:][3] == 1, np.c_[receive_center_list,result])
        detected_legs_pair = []

        for k in detected_legs:
            POINTS.append([k[0],k[1],0,0xff00ff])

        #足のペア検索
        for k in range(len(detected_legs)-1):
            l = k+1
            while l < len(detected_legs):
                point_difference = em.distanceBetweenTwoPoints(detected_legs[k],detected_legs[l])
                if 0.15 <= point_difference and point_difference <= 0.35 :
                    #x,y,z,足番号1,足番号2
                    detected_legs_pair.append([(detected_legs[k][0]+detected_legs[l][0])/2,(detected_legs[k][1]+detected_legs[l][1])/2,0,[detected_legs[k],detected_legs[l]]])
                l+=1

        #人間の可能性がある地点
        if len(detected_legs_pair) > 0:
            for dlp in detected_legs_pair:
                POINTS.append([dlp[0],dlp[1],dlp[2],0x00ff00])
                point_difference = em.distanceBetweenTwoPoints(dlp,kf_status_x)
                if min_difference > point_difference and point_difference < prediction_range:
                    min_difference = point_difference
                    prediction_point = dlp

            #人間の可能性がある地点
            if min_difference != 50:
                POINTS.append([prediction_point[0],prediction_point[1],prediction_point[2],0xff0000])
                xv = (prediction_point[0]-former_prediction_point[0])/T
                yv = (prediction_point[1]-former_prediction_point[1])/T
                former_prediction_point = [prediction_point[0],prediction_point[1]]
                number_list.data = [0,prediction_point[0],prediction_point[1]]
                former_T = 0
                swb1 = True
    #ない場合
    else:
        print("No data")


    if not swb1:
        number_list.data = [1,0,0]
        if former_T >= 2:
            xv,yv = 0,0
            former_prediction_point = [0.5,0]
            kf = KalmanFilter(x_0, p_0, A, b, Q, R, u)
        else:
            former_T += lap_T

    #配信
    point_cloud = pc2.create_cloud(HEADER, FIELDS, POINTS)
    pub.publish(point_cloud)
    pub_status.publish(number_list)


if __name__ == '__main__':
    feature_list = []
    with open(os.environ.get("CARTBOT_FEATURES_DATA_FOLDER")+'/leg_features_data.csv', 'r') as f:
        reader = csv.reader(f)
        for r in reader:
            feature_list.append([r[0],r[1],r[2]])

    clf_a = svm.OneClassSVM(nu=0.1,kernel='rbf',gamma="auto")
    clf_a.fit(feature_list)

    #listner
    rospy.init_node('detection_leg', anonymous=True)
    rospy.Subscriber("lrf_feature", String, callback)
    pub = rospy.Publisher('/center_point', PointCloud2, queue_size=1)
    pub_status = rospy.Publisher('status', Float32MultiArray, queue_size=1)
    r = rospy.Rate(20)
    #追跡クラス
    feature_number = 3

    #カルマンフィルター
    x_0 = np.array([0.5, 0])
    p_0 = np.array([[1, 0.5],
                   [0, 1]]) 
    #解釈: https://qiita.com/IshitaTakeshi/items/740ac7e9b549eee4cc04 基準
    A = np.array([[1, 0],
                  [0, 1]]) #Ft ?
    b = np.array([0, 0]) #Bt ?
    Q = np.array([[1, 0],
                  [0, 1]]) #
    R = np.array([[1, 0],
                  [0, 1]]) #謎
    u = np.array([[1,0],[0,1]])

    kf = KalmanFilter(x_0, p_0, A, b, Q, R, u)

    #人座標の位置
    former_T = 0
    xv,yv = 0,0
    former_prediction_point = [0.5,0]
    prediction_range = 0.2

    tw = TimeWatcher()
    tw.start()

    while not rospy.is_shutdown():
        r.sleep()
