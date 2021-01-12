#!/usr/bin/env python
# coding:utf-8

import re
import csv
import time
import os

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

#pointcloud2変数
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

def callback(data):

    global former_prediction_point
    global xv,yv
    global prediction_range
    global former_T
    global kf

    lap_T = tw.lap()
    T = former_T + lap_T

    #kf.A = np.array([[1.5,T*1.5],[0,1.5]])
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
    swb1 = False

    #受信データがある場合とない場合
    #ある場合
    if len(receive_float_list) > 0:
        receive_float_list = np.array(receive_float_list)
        #受信値の特徴データ
        receive_feature_list = receive_float_list[:,:3]
        #受信値の中心座標データ
        receive_center_list = receive_float_list[:,3:]
        #人間検知
        result = clf_a.predict(receive_feature_list)
        #print(receive_feature_list)
        #応急処置、中心点を始めに検知した方に向かう
        min_difference = 50
        #人間として検知したものの中心点をリスト化
        kf_status_x,kf_status_p = kf.status()
        POINTS.append([kf_status_x[0],kf_status_x[1],0,0xffff00])
        for r,rcl in zip(result,receive_center_list):
            if r == 1:
                POINTS.append([rcl[0],rcl[1],rcl[2],0x00ff00])
                point_difference = em.distanceBetweenTwoPoints(rcl,kf_status_x)
                if min_difference > point_difference and point_difference < prediction_range:
                    min_difference = point_difference
                    prediction_point = rcl

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
    with open(os.environ.get("CARTBOT_FEATURES_DATA_FOLDER")+'/torso_features_data.csv', 'r') as f:
        reader = csv.reader(f)
        for r in reader:
            feature_list.append([r[0],r[1],r[2]])

    clf_a = svm.OneClassSVM(nu=0.1,kernel='rbf',gamma="auto")
    clf_a.fit(feature_list)

    #listner
    rospy.init_node('detection_torso', anonymous=True)
    rospy.Subscriber("lrf_feature", String, callback)
    pub = rospy.Publisher('/center_point', PointCloud2, queue_size=1)
    pub_status = rospy.Publisher('status', Float32MultiArray, queue_size=1)
    r = rospy.Rate(20)
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
    tw = TimeWatcher()
    tw_spec = TimeWatcher()
    tw.start()
    xv,yv = 0,0
    former_prediction_point = [0.5,0]
    prediction_range = 0.2
    former_T = 0
    tw_spec.start()

    while not rospy.is_shutdown():
        r.sleep()
        #print("time:"str(tw_spec.lap()))
