#!/usr/bin/env python
import json
import time
# TODO: uncomment this when used as a ROS node
import rospy
# import carma_srvs
from std_msgs.msg import String, Header, Bool
from beginner_tutorials.msg import ExternalObjectList, CarlaEgoVehicleControl, ExternalObject, VehicleStatus, CarmaInit
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseWithCovariance, Twist, TwistWithCovariance, Vector3
from math import cos, sin
from random import random

def receiveCarmaInit(data):
    print("Initialization Data:")
    print(data)

##### TODO: Setup subscribers and publishers for each of the exchanged messages to ROS
rospy.init_node('CARMA_Interface_init')
# # subscribe to a topic using rospy.Subscriber class
sub=rospy.Publisher('IsReady_Topic', Bool, queue_size=1)
# #publish messages to a topic using rospy.Publisher class
pub_vs=rospy.Subscriber('CarmaInit_Topic', CarmaInit, receiveCarmaInit)

rate = rospy.Rate(0.25) # 1hz
count = 0
while True:
    toSend = Bool()
    toSend.data = False
    if count > 5:
        toSend.data = True
    count += 1
    print('Sending from CARMA')
    sub.publish(toSend)
    print(toSend)
    rate.sleep()

