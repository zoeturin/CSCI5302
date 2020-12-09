#!/usr/bin/env python2

import rospy
from EKF import *
from ros_service import *
from std_msgs.msg import String, Bool
from webots_ros.msg import RecognitionObject
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D, Point
from simple_control.msg import ControlCommand
from localization.msg import SLAMFeature
from webots_ros.srv import get_float, automobile_get_dimensions
'''
TODO:

'''

class vehicle_params(object):
    def __init__(self):
        # Services:
        get_dimensions = service(vehicle_name+"/automobile/get_dimensions", automobile_get_dimensions)
        # Service calls:
        dims = get_dimensions.srv()
        self.TRACK = dims.trackRear
        self.BASE = dims.wheelBase
        self.WHEEL = dims.frontWheelRadius
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
    rospy.logwarn("Got vehicle name")

def feature_callback(feature_obj):
    global feature_buffer
    feature_pos = feature_obj.position
    theta = solver.x_hat[2]
    state = np.array([-np.cos(theta)*feature_pos.x, -np.cos(theta)*feature_pos.z])+solver.x_hat[0:2]
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
    global solver
    th_dot = data.angular_velocity.y
    cov = data.angular_velocity_covariance
    solver.gyro_cov = cov[4]

def control_callback(data):
    global u
    u = [data.throttle, data.brake, data.steering_angle, data.gear]

def init_vehicle_model():
    pass

def main():
    while vehicle_name is None:
        pass
    global feature_buffer, solver, z, u
    # Services:
    #get_time_step = service(vehicle_name+"/robot/get_basic_time_step", get_float)
    #rospy.logwarn('got services')
    # Service calls:
    #timestep = get_time_step.srv(True)
    timestep = .01 # in sec, hardcoded bc get_basic_time_step requires world node running in ros
    # Subscribers:
    rospy.Subscriber(vehicle_name+"/front_camera/recognition_objects", RecognitionObject, feature_callback)
    rospy.Subscriber(vehicle_name+"/gps/values", NavSatFix, gps_callback)
    rospy.Subscriber(vehicle_name+"/gyro/values", Imu, gyro_callback)
    rospy.Subscriber("control_command", ControlCommand, control_callback)
    # Publishers:
    state_pub = rospy.Publisher('state_estimate', Pose2D, queue_size=10)
    feature_pub = rospy.Publisher('features', SLAMFeature, queue_size=50)
    road_pub = rospy.Publisher('road_features', SLAMFeature, queue_size=50)
    loc_ready_pub = rospy.Publisher('localization_is_ready', Bool, queue_size=10)
    # Vehicle model:
    params = vehicle_params()
    model = dynamicsModel(params.car_dict)
    solver.set_model(model)
    rospy.logwarn('EKF is ready')
    while not rospy.is_shutdown():
        rospy.logwarn('prediction')
        solver.predict(u,timestep)
        if len(feature_buffer) != 0:
            rospy.logwarn('features')
            z = z.extend(feature_buffer)
            feature_buffer = []
        rospy.logwarn('update')
        solver.update(z)
        z = z[0:3] # remove features
        pose = Pose2D(x=solver.x[0],y=solver.x[1],theta=solver.x[2])
        rospy.logwarn('publish state')
        state_pub.publish(pose)
        for feature in solver.known_features:
            pos = Point(x=feature.state[0],y=0,z=feature.state[1])
            feature_msg = SLAMFeature(position=pos, model=feature.model, feature_id=feature.number)
            feature_pub.publish(feature_msg)
            if feature.model == 'road':
                road_pub.publish(feature_msg)

vehicle_name = None
feature_buffer = []
z = np.zeros(3)
u = [0.,0.,0.,0]
solver = EKF_solver()
rospy.init_node('controller')
rospy.Subscriber("model_name", String, vehicle_name_callback)
main()
