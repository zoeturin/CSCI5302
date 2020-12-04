#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
#from controller import Robot, Camera
# from vehicle import Driver, Car
import numpy as np
from webots_ros.srv import *
#from simple_control.srv import *

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
    enable_rear_camera = service("/rear_camera/enable", set_int)
    enable_lidar = service("/Sick_LMS_291/enable", set_int)
    enable_point_cloud = service("/Sick_LMS_291/enable_point_cloud", set_bool)
    enable_gps = service("/gps/enable", set_int)

    rospy.loginfo("All services ready")

    # Initial service calls
    # Control
    set_brake_intensity.srv(0)
    set_gear.srv(1)
    # Misc
    dims = get_dimensions.srv()
    timestep = get_time_step.srv()
    # Sensors
    enable_front_camera.srv(30)
    #enable_lidar.srv(timestep)
    enable_point_cloud.srv(True)


    while True: #robot.step() != -1:
        # pc = lidar.getPointCloud();
        try
        set_throttle.srv(1)



# create the Robot instance.
#robot = Driver();
# robot = Car()
# front_camera = robot.getCamera("front_camera")
# rear_camera = robot.getCamera("rear_camera")
# lidar = robot.getLidar('Sick LMS 291');
#
# # get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
#
# MAX_STEER = 1;
# STEER_SPEED = 10;
# WHEELBASE = robot.getWheelbase()
# WHEELTRACK = robot.getTrackFront()
# MIN_TURN_RADIUS = WHEELBASE/np.tan(MAX_STEER) + WHEELTRACK/2;
# MAX_SPEED = 138.889;
vehicle_name = None
rospy.init_node('controller', anonymous=True)
rospy.Subscriber("model_name", String, callback)
#pub = rospy.Publisher(vehicle_name + '/automobile/motor', Float64, queue_size=10)
#rospy.spin()
main()
