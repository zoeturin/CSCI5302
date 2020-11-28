#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
# from controller import Robot, Camera
# from vehicle import Driver, Car
import numpy as np
from webots_ros.srv import *


def callback(data):
    global vehicle_name
    vehicle_name = data.data
    rospy.loginfo("Got vehicle name")

def main():

    # #robot.__init__();
    # front_camera.enable(30)
    # rear_camera.enable(30)
    # lidar.enable(timestep);
    # lidar.enablePointCloud();
    # print("lidar freq: "+str(lidar.getFrequency()));
    #
    # #robot.setSteeringAngle(0.5);
    # robot.setThrottle(1);
    # robot.setBrakeIntensity(0);
    # robot.setGear(1); # starts in 0 (neutral)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while vehicle_name is None:
        pass
    rospy.loginfo("Waiting for services")
    rospy.loginfo("Vehicle name:" + vehicle_name)
    rospy.wait_for_service(vehicle_name + "/automobile/set_throttle")
    rospy.wait_for_service(vehicle_name + "/automobile/set_gear")
    rospy.loginfo("Got services")
    while True: #robot.step() != -1:
        # pc = lidar.getPointCloud();
        try:
            gear_srv = rospy.ServiceProxy(vehicle_name+'/automobile/set_gear', set_int)
            throttle_srv = rospy.ServiceProxy(vehicle_name+'/automobile/set_throttle', set_float)
            gear_srv(1)
            throttle_srv(1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # Enter here exit cleanup code.
    #robot.__del__();


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
