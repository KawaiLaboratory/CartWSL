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

model = load_model("/home/daidai/catkin_ws/src/detected_human/scripts/detected_human_model.h5")
MAX_RANGE = 6
DATASIZE  = 725

x_desire = 0
y_desire = 0

def callback(data):
  l_data = pd.DataFrame(data.ranges).fillna(0)
  l_data = np.reshape(l_data["l"].values/MAX_RANGE)
  result = model.predict(l_data)
  print(result)

def detect_torso():
  rospy.init_node("detected_human")
  rospy.Subscriber("/scan", LaserScan, callback)
  print("ok")
  rospy.spin()

if __name__ == '__main__':
  detect_torso()
