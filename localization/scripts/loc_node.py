#!/usr/bin/env python2

import rospy
from EKF import *

from ros_service import *
from std_msgs.msg import String, Bool
from webots_ros.msg import RecognitionObject
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D, Point
from simple_control.msg import ControlCommand
from localization.msg import SLAMFeature, SLAMFeatureArray
from webots_ros.srv import get_float, set_int #, automobile_get_dimensions
from simple_control.srv import automobile_get_dimensions
'''
TODO:

'''

class vehicle_params(object):
    def __init__(self):
        # Services:
        get_dimensions = service(vehicle_name+"/automobile/get_dimensions", automobile_get_dimensions)
        # Service calls:
        dims = get_dimensions.srv()
        # print('got dims')
        self.TRACK = dims.trackRear
        self.BASE = dims.wheelBase
        self.WHEEL = dims.frontWheelRadius
        # self.TRACK = 1.72
        # self.BASE = 2.94
        # self.WHEEL = .36
        self.create_car_dict()

    def create_car_dict(self):
        car_dict = {
            "engineMaxTorque": 639,
            "engineMaxPower": 147000,
            "wheelRadius": self.WHEEL,
            "wheelbase": self.BASE,
            "wheeltrack": self.TRACK,
            "gearRatios": [-6.3, 9],
            "brakeCoefficient": 2000,
            "centerOfMass": [0, 0, 1.27],
            "mass": 1847,
            "momentOfInertia": [[26116.9, 25854, 4466.54], [2.33023e-5, 0.000158947, -3204.91]]
        }
        self.car_dict = car_dict

def vehicle_name_callback(name):
    global vehicle_name
    vehicle_name = name.data
    rospy.loginfo("Got vehicle name")

def feature_callback(feature_obj):
    global feature_buffer
    feature_pos = feature_obj.position
    #print(feature_pos)
    theta = solver.x_hat[2]
    state = np.array([-np.cos(theta)*feature_pos.x, -np.cos(theta)*feature_pos.z])+solver.x_hat[0:2]
    #print(state)
    feature_new = feature(feature_obj.model, state)
    feature_buffer.append(feature_new)
    rospy.loginfo("Got feature")

def gps_callback(data):
    global z, solver
    z[0] = data.latitude # x pos
    z[1] = data.longitude # z pos
    cov = data.position_covariance
    xz_cov = np.zeros((2,2))
    xz_cov[0,0] = cov[0]
    xz_cov[1,1] = cov[4]
    xz_cov[0,1] = cov[2]
    xz_cov[1,0] = cov[2]
    solver.gps_cov = xz_cov
    pass

def gyro_callback(data):
    global solver, z
    th_dot = data.angular_velocity.y
    z[2] = th_dot
    cov = data.angular_velocity_covariance
    solver.gyro_cov = cov[4] + .0001

def control_callback(data):
    global u
    u = [data.throttle, data.brake, data.steering_angle, data.gear]

def main():
    while vehicle_name is None:
        pass
    global feature_buffer, solver, z, u
    #timestep = .01 # in sec, hardcoded bc get_basic_time_step requires world node running in ros
    # Subscribers:
    rospy.Subscriber(vehicle_name+"/front_camera/recognition_objects", RecognitionObject, feature_callback)
    rospy.Subscriber(vehicle_name+"/gps/values", NavSatFix, gps_callback)
    rospy.Subscriber(vehicle_name+"/gyro/values", Imu, gyro_callback)
    rospy.Subscriber("control_command", ControlCommand, control_callback)
    # Publishers:
    state_pub = rospy.Publisher('state_estimate', Pose2D, queue_size=10)
    feature_pub = rospy.Publisher('features', SLAMFeatureArray, queue_size=10)
    #road_pub = rospy.Publisher('road_features', SLAMFeature, queue_size=50)
    loc_ready_pub = rospy.Publisher('localization_is_ready', Bool, queue_size=10)
    # Services:
    get_time_step = service(vehicle_name+"/robot/get_basic_time_step", get_float)
    time_step = service(vehicle_name+"/robot/time_step", set_int)
    rospy.loginfo('got services')
    # Vehicle model:
    params = vehicle_params()
    model = dynamicsModel(params.car_dict)
    solver.set_model(model)

    # Service calls:
    timestep = get_time_step.srv()
    timestep = timestep.value
    # timestep = 10
    timestep_sec = timestep/1000.
    rospy.loginfo('EKF is ready')
    loc_ready_pub.publish(True)
    while not rospy.is_shutdown():
        # Predict:
        rospy.loginfo('prediction')
        solver.predict(u,timestep_sec)
        #pose = Pose2D(x=solver.x_hat[0],y=solver.x_hat[1],theta=solver.x_hat[2])
        #state_pub.publish(pose)
        # Update:
        if len(feature_buffer) != 0:
            features = copy.copy(feature_buffer)
            rospy.loginfo('features')
            z = np.concatenate([z,features])
            feature_buffer = []
        rospy.loginfo('update')
        solver.update(z)
        z = z[0:3] # remove features
        # Publish:
        pose = Pose2D(x=solver.x[0],y=solver.x[1],theta=solver.x[2])
        rospy.loginfo('publish state')
        if np.isnan(pose.x) or np.isnan(pose.y):
            rospy.logfatal('State estimate is NaN')
            raise Exception('State estimate is Nan')
        state_pub.publish(pose)
        feature_array = []
        for feature in solver.known_features:
            pos = Point(x=feature.state[0],y=0,z=feature.state[1])
            feature_array.append(SLAMFeature(position=pos, model=feature.model, feature_id="%s"%feature.number))
        feature_pub.publish(SLAMFeatureArray(feature_array=feature_array))
        #time_step.srv(int(timestep))


vehicle_name = None
feature_buffer = []
z = np.zeros(3)
u = [0.,0.,0.,0]
solver = EKF_solver()
rospy.init_node('controller')
rospy.Subscriber("model_name", String, vehicle_name_callback)
main()
