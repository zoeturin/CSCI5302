#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import numpy as np
from webots_ros.srv import *
#from simple_control.srv import *
from simple_control.msg import ControlCommand

class service(object):
    def __init__(self, path, srv_type):
        self.path = vehicle_name + path
        self.type = srv_type
        rospy.loginfo("Waiting for service %s" %self.path)
        rospy.wait_for_service(self.path)
        self.srv = rospy.ServiceProxy(self.path, self.type)
        rospy.loginfo("Got service %s" %self.path)

def callback(data):
    global vehicle_name
    vehicle_name = data.data
    rospy.loginfo("Got vehicle name")


def main():
    while vehicle_name is None:
        pass
    rospy.loginfo("Vehicle name: " + vehicle_name)

    # Services
    # Control
    set_throttle = service("/automobile/set_throttle", set_float)
    set_brake_intensity = service("/automobile/set_brake_intensity", set_float)
    set_gear = service("/automobile/set_gear", set_int)
    set_steering_angle = service("/automobile/set_steering_angle", set_float)
    # Misc
    get_dimensions = service("/automobile/get_dimensions", automobile_get_dimensions)
    get_time_step = service("/robot/get_basic_time_step", get_float)
    # Sensors
    enable_front_camera = service("/front_camera/enable", set_int)
    enable_recognition = service("/front_camera/recognition_enable", set_int)
    enable_rear_camera = service("/rear_camera/enable", set_int)
    enable_lidar = service("/Sick_LMS_291/enable", set_int)
    enable_point_cloud = service("/Sick_LMS_291/enable_point_cloud", set_bool)
    enable_gps = service("/gps/enable", set_int)
    enable_gyro = service("/gyro/enable", set_int)

    rospy.loginfo("All services ready")

    # Initial service calls
    # Control
    set_brake_intensity.srv(0)
    set_gear.srv(1)
    # Misc
    dims = get_dimensions.srv()
    TRACK = dims.trackRear
    BASE = dims.wheelBase
    WHEEL = dims.frontWheelRadius
    timestep = get_time_step.srv()
    # Sensors
    enable_front_camera.srv(30)
    enable_recognition.srv(1)
    enable_gps.srv(1)
    enable_gyro.srv(1)
    #enable_lidar.srv(timestep)
    enable_point_cloud.srv(True)

    # Publishers:
    control_pub = rospy.Publisher('control_command', ControlCommand, queue_size=10)

    while not rospy.is_shutdown():
        # u = [throttle, brake, desired steering angle, desired gear]
        try:
            throttle = 1
            brake = 0
            steering_angle = 0
            gear = 1
            u = ControlCommand(throttle=throttle, brake=brake, steering_angle=steering_angle, gear=gear)
            control_pub.publish(u)
            set_throttle.srv(throttle)
            set_brake_intensity.srv(brake)
            set_steering_angle.srv(steering_angle)
            set_gear.srv(gear)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

vehicle_name = None
rospy.init_node('controller', anonymous=True)
rospy.Subscriber("model_name", String, callback)
main()
