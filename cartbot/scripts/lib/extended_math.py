#!/usr/bin/env python
# coding:utf-8
#参照 https://www.hellocybernetics.tech/entry/2018/03/08/131445 https://satomacoto.blogspot.com/2011/06/python.html

import rospy
import numpy as np
import math

def distanceBetweenTwoPoints(coordinate1,coordinate2):
    return math.sqrt((coordinate1[0] - coordinate2[0])**2 + (coordinate1[1] - coordinate2[1])**2)

def twoVectorAngle(A1,B1,A2,B2):
    a = [B1[0]-A1[0],B1[1]-A1[1]]
    b = [B2[0]-A2[0],B2[1]-A2[1]]
    
    return math.acos((a[0]*b[0]+a[1]*b[1])/(math.sqrt(a[0]**2+a[1]**2)*math.sqrt(b[0]**2+b[1]**2)))

def xyAngle(x,y):
    if x != 0:
        rad = math.atan(y/x)
    else:
        rad = 0

    return rad

def xyDistance(x,y):
    return math.sqrt(x**2+y**2)