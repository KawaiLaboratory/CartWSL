#!/home/daidai/learning/tf12/bin/python
#-*- coding: utf-8 -*-

import roslib
import rospy
import h5py
import numpy as np
import pandas as pd
from keras.models import load_model
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class deepLearnModel():
  def __init__(self):
    self.sub = rospy.Subscriber("/scan", LaserScan, callback)
    self.pub = rospy.Publisher("/human_point", Float32MultiArray, queue_size=10)
    self.MAX_RANGE = 6
    self.DATASIZE  = 726
    self.model = load_model("/home/daidai/catkin_ws/src/detected_human/scripts/detected_human_model.h5")

  def callback(self, data):
    l_np = np.nun_to_num(data.ranges)
    l_np = np.resize(l_np, (1, self.DATASIZE, 1))
    pre = model.predict(l_np/self.MAX_RANGE)
    pre = np.reshape(pre, -1)
    x_desire = pre[0]*np.cos(pre[1])
    y_desire = pre[0]*np.sin(pre[1])

    msg = Float32MultiArray()
    msg.data = np.asarray([x_desire, y_desire])
    self.pub.publish(msg)

if __name__ == '__main__':
  rospy.init_node("detected_human")
  rospy.spin()
