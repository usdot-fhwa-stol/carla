#!/usr/bin/env python
import json
import time
# TODO: uncomment this when used as a ROS node
import rospy
# import carma_srvs
from std_msgs.msg import String, Header
from beginner_tutorials.msg import ExternalObjectList, CarlaEgoVehicleControl, ExternalObject, VehicleStatus
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseWithCovariance, Twist, TwistWithCovariance, Vector3
from math import cos, sin
from random import random

rx_flag_status = False
rx_flag_pose = False
rx_flag_ext = False

def receiveVehicleStatus(data):
    global rx_flag_status
    rx_flag_status = True
    print("Vehicle Status:")
    #print(data.header.stamp, data.speed)

def receivePoseStamped(data):
    global rx_flag_pose
    print("Pose Stamped:")
    rx_flag_pose = True
    #print(data.header.stamp, data.pose)

def receiveExternalObjectList(data):
    global rx_flag_ext
    print("External Object List:")
    rx_flag_ext = True
    #print(data.header.stamp, data.objects)

##### TODO: Setup subscribers and publishers for each of the exchanged messages to ROS
rospy.init_node('CARMA_Interface_2')
# # subscribe to a topic using rospy.Subscriber class
#sub=rospy.Publisher('sub', CarlaEgoVehicleControl, queue_size=1)
sub=rospy.Publisher('CarlaEgoVehicleControl_Topic', CarlaEgoVehicleControl, queue_size=1)
# #publish messages to a topic using rospy.Publisher class
pub_vs=rospy.Subscriber('VehicleStatus_Topic', VehicleStatus, receiveVehicleStatus)
pub_pose=rospy.Subscriber('PoseStamped_Topic', PoseStamped, receivePoseStamped)
pub_eol=rospy.Subscriber('ExternalObjectList_Topic', ExternalObjectList, receiveExternalObjectList)

rate = rospy.Rate(20) # 1hz
while True:
#    while not rx_flag_status or not rx_flag_pose or not rx_flag_ext:
#        rate.sleep()
#    rx_flag_status, rx_flag_pose, rx_flag_ext = False, False, False
    print("            All received\n")
    toSend = CarlaEgoVehicleControl()
#    toSend.throttle = random()
    toSend.throttle = 0.5
    toSend.steer = random()
    toSend.brake = 0
    print("Sending from CARMA, throttle:",toSend.throttle,", brake:",toSend.brake,", steer:",toSend.steer)
    sub.publish(toSend)
    rate.sleep()

