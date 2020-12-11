#!/usr/bin/env python2

import rospy
import numpy as np
import lane_detector as ld
from std_msgs.msg import String
from webots_ros.srv import *
from webots_ros.msg import Float64Stamped
#from simple_control.srv import *
from simple_control.msg import ControlCommand
from sensor_msgs.msg import Image

class service(object):
    def __init__(self, path, srv_type):
        self.path = vehicle_name + path
        self.type = srv_type
        rospy.loginfo("Waiting for service %s" %self.path)
        rospy.wait_for_service(self.path)
        self.srv = rospy.ServiceProxy(self.path, self.type)
        rospy.loginfo("Got service %s" %self.path)

def vehicle_name_callback(data):
    global vehicle_name
    vehicle_name = data.data
    rospy.loginfo("Got vehicle name")

def steering_angle_callback(data):
    global steering_angle
    steering_angle = data.data

def throttle_callback(data):
    global throttle
    throttle = data.data

def image_callback(data):
    global camera_image
    camera_image = data.data

def main():
    global steering_angle, throttle, vehicle_name, camera_image

    while vehicle_name is None:
        pass
    rospy.loginfo("Vehicle name: " + vehicle_name)

    # Publishers:
    control_pub = rospy.Publisher('control_command', ControlCommand, queue_size=10)
    # Subscribers:
    rospy.Subscriber(vehicle_name+"/automobile/steering_angle", Float64Stamped, steering_angle_callback)
    rospy.Subscriber(vehicle_name+"/front_camera/image", Image, image_callback)
    rospy.Subscriber(vehicle_name+"/automobile/throttle", Float64Stamped, throttle_callback)

    # Services
    # Control
    set_throttle = service("/automobile/set_throttle", set_float)
    set_brake_intensity = service("/automobile/set_brake_intensity", set_float)
    set_gear = service("/automobile/set_gear", set_int)
    set_steering_angle = service("/automobile/set_steering_angle", set_float)
    set_cruising_speed = service("/automobile/set_cruising_speed", set_float)
    # Misc
    get_dimensions = service("/automobile/get_dimensions", automobile_get_dimensions)
    get_time_step = service("/robot/get_basic_time_step", get_float)
    # Front camera
    enable_front_camera = service("/front_camera/enable", set_int)
    enable_recognition = service("/front_camera/recognition_enable", set_int)
    camera_save_image = service("/front_camera/save_image", save_image)
    get_info = service("/front_camera/get_info", camera_get_info)
    # Other sensors
    enable_rear_camera = service("/rear_camera/enable", set_int)
    enable_lidar = service("/Sick_LMS_291/enable", set_int)
    enable_point_cloud = service("/Sick_LMS_291/enable_point_cloud", set_bool)
    enable_gps = service("/gps/enable", set_int)
    enable_gyro = service("/gyro/enable", set_int)
    rospy.loginfo("All services ready")

    # Initial service calls
    # Misc
    dims = get_dimensions.srv()
    TRACK = dims.trackRear
    BASE = dims.wheelBase
    WHEEL = dims.frontWheelRadius
    #timestep = get_time_step.srv()
    timestep = 10 # hard code if not running webots from launch file
    # Front camera
    enable_front_camera.srv(30)
    enable_recognition.srv(1)
    info = get_info.srv()
    camera_width = info.width
    camera_height = info.height
    # Other sensors
    enable_gps.srv(1)
    enable_gyro.srv(1)
    enable_lidar.srv(timestep)
    enable_point_cloud.srv(True)
    # Control
    set_brake_intensity.srv(0)
    set_gear.srv(1)
    set_throttle.srv(.5)

    while camera_image is None:
        pass
    while not rospy.is_shutdown():
        front_img = np.frombuffer(camera_image, np.uint8).reshape((camera_height, camera_width, 4))
        # no idea where this saves the image:
        # save_result = camera_save_image.srv(filename="~/front_img1.jpg", quality=75)
        #print(save_result)
        print('curr steer cmd', steering_angle)
        #yaw = ld.get_steer_cmd("front_img1.jpg", steering_angle)
        yaw = ld.get_steer_cmd(front_img, steering_angle, frame_is_array=True)
        print("yaw: ", yaw)
        steering_cmd = (yaw-steering_angle) * 0.7
        set_steering_angle.srv(steering_cmd)

        cruising_speed = 30
        set_cruising_speed.srv(cruising_speed)
        # u = [throttle, brake, desired steering angle, desired gear]
        u = ControlCommand(throttle=throttle, brake=0, steering_angle=steering_cmd, gear=1)
        control_pub.publish(u)

camera_image = None
vehicle_name = None
steering_angle = 0
throttle = 0
rospy.init_node('controller', anonymous=True)
rospy.Subscriber("model_name", String, vehicle_name_callback)
main()
