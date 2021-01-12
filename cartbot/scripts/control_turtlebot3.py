#!/usr/bin/env python
# coding:utf-8

import os

#ROS
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

#self made
from lib.follower3 import Follower3
import lib.extended_math as em
from lib.time_watcher import TimeWatcher

#配信用変数 --ここから--
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

RECORD_HEADER = Header(frame_id='/recorder')
RECORD_POINTS = []

def callback(data):
    global status
    status = data.data
    print(status)
    if status[0] == 0:
        follower.target_coord_update(status[1],status[2])
    #print("time:"+str(tw.lap()))

def position(msg):
    global odom
    odom = [msg.pose.pose.position.x,msg.pose.pose.position.y]

if __name__ == '__main__':

    with open(os.environ.get("CATKIN_FOLDER")+'/status0.csv', 'w') as f0, open(os.environ.get("CATKIN_FOLDER")+'/robot_odom.csv', 'w') as f1, open(os.environ.get("CATKIN_FOLDER")+'/target_odom.csv', 'w') as f2:
        rospy.init_node('control_turtlebot', anonymous=True)
        rospy.Subscriber("status", Float32MultiArray, callback)
        rospy.Subscriber("odom",Odometry,position)
        pub = rospy.Publisher('/estimate_point', PointCloud2, queue_size=1)
        record_pub = rospy.Publisher('/record_point', PointCloud2, queue_size=1)
        r = rospy.Rate(20)
        status = [1,0,0,0,0]
        #追跡クラス
        follower = Follower3(0.1,1.9,1.0,1.2)
        tw = TimeWatcher()
        tw.start()
        odom = [0,0]
        start = False

        while not rospy.is_shutdown() and odom[0] <= 10:
            r.sleep()
            print(status[0])
            if status[0] == 0:
                start = True
            if start:
                RECORD_POINTS.append([odom[0],odom[1],0,0xffff00])
                if status[0] == 0:
                    RECORD_POINTS.append([odom[0]+status[1],odom[1]+status[2],0,0xff0000])
                f0.write(str(status[0])+'\n')
                f1.write(str(odom[0])+","+str(odom[1])+"\n")
                f2.write(str(odom[0]+status[1])+","+str(odom[1]+status[2])+"\n")
            follower.move(status[0])
            POINTS = []
            target_coord = follower.target_coordinate_view()
            #print(target_coord)
            POINTS.append([target_coord[0],target_coord[1],0,0xee84ff])
            point_cloud = pc2.create_cloud(HEADER, FIELDS, POINTS)
            pub.publish(point_cloud)
            record_point_cloud = pc2.create_cloud(RECORD_HEADER, FIELDS, RECORD_POINTS)
            record_pub.publish(record_point_cloud)