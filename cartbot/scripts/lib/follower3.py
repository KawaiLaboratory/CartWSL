#!/usr/bin/env python
# coding:utf-8

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist

from lib.time_watcher import TimeWatcher
import lib.extended_math as em


class Follower3:

    #近接限界距離,活動開始距離
    stop_distance,start_distance = 1.0,0.8
    #目標座標
    target_coordinate,target_angle = [0,0],0
    #経過時間(見失った際のみ利用)
    past_times = 0
    #前速度、角速度
    former_v,former_omega = 0,0
    #ターゲットを見失った際のアングルと距離の到達是非 distance,angle
    status1_status = [0,0]
    #タートルボットへデータ送信
    cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    #経過時間計測
    tw = TimeWatcher()
    #速度と角速度のゲイン
    kt,kr = 100000.0,1.9

    def __init__(self,kt,kr,stop_distance,start_distance):
        self.kt = kt
        self.kr = kr
        self.stop_distance = stop_distance
        self.start_distance = start_distance
        self.tw.start()

    def target_coord_update(self,x,y):
        self.target_coordinate = [x,y]

    def target_coordinate_view(self):
        return self.target_coordinate

    def status1_Initialization(self):
        self.status1_status = [False,False]
        self.target_angle = 0
        self.past_times = 0
        self.former_v,former_omega = 0,0
        

    def move(self,status):
        move_cmd = Twist()
        t = self.tw.lap()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0
        #---------------見失っていない場合------------------
        if status == 0:
            x = self.target_coordinate[0]
            y = self.target_coordinate[1]
            self.status1_Initialization()
            distance = em.xyDistance(x,y)
            rad = em.xyAngle(x,y)
            self.target_angle = rad
            deg = math.degrees(rad)
            if distance >= self.start_distance:
                move_cmd.linear.x = self.kt * distance * math.cos(rad)
                self.former_v = move_cmd.linear.x
            if deg <= -5 or 5 <= deg:
                move_cmd.angular.z = self.kr * distance * math.sin(rad)
                self.former_omega = move_cmd.angular.z

        #-------------------見失った場合----------------
        elif status == 1:
            s = self.former_v * t
            delta_theta = t * self.former_omega
            if not self.status1_status[0]:
                self.target_coordinate[0] = self.target_coordinate[0] - s * math.cos(delta_theta / 2)
                self.target_coordinate[1] = self.target_coordinate[1] - s * math.sin(delta_theta / 2)
            if not self.status1_status[1]:
                self.target_angle = self.target_angle - delta_theta
            deg = math.degrees(self.target_angle)
            distance = em.xyDistance(self.target_coordinate[0],self.target_coordinate[1])
            if distance >= self.start_distance:
                move_cmd.linear.x = self.kt * distance * math.cos(self.target_angle)
                self.former_v = move_cmd.linear.x
            else:
                self.status1_status[0] = True
            if deg <= -5 or 5 <= deg:
                move_cmd.angular.z = self.kr * distance * math.sin(self.target_angle)
                self.former_omega = move_cmd.angular.z
            else:
                self.status1_status[1] = True
            
            if self.status1_status[1] and self.status1_status[0]:
                print("Waiting...")

        self.cmd_vel.publish(move_cmd)