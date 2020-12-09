import rospy
from EKF.py import *
from webots_ros.msg import RecognitionObject
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D, Point
from simple_control.msg import ControlCommand
from localization.msg import SLAMFeature
from webots_ros.srv import get_float
'''
TODO:
- add publisher for features (model, position)

'''
def vehicle_name_callback(name):
    global vehicle_name
    vehicle_name = name.data
    rospy.loginfo("Got vehicle name")

def feature_callback(feature_obj):
    global feature_buffer
    feature_pos = feature_obj.position
    theta = solver.x_hat[2]
    state = np.array([-np.cos(theta)*feature_pos.x, -np.cos(theta)*feature_pos.z])+solver.x_hat[0:2]
    feature = feature(feature_obj.model, state)
    feature_buffer.append(feature)
    rospy.loginfo("Got feature")

def gps_callback(data):
    global z, solver
    z[0] = data.latitude # x pos
    z[1] = data.longitude # z pos
    cov = data.position_covariance
    xz_cov = np.zeros(2)
    xz_cov[0][0] = cov[0]
    xz_cov[1][1] = cov[4]
    xz_cov[0][1], xz_cov[1][0] = cov[2]
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

def main():
    while vehicle_name is None:
        pass
    # Services:
    get_time_step_srv = rospy.ServiceProxy(vehicle_name+"/robot/get_basic_time_step", get_float)
    rospy.wait_for_service(vehicle_name+"/robot/get_basic_time_step")
    # Subscribers:
    rospy.Subscriber(vehicle_name+"/front_camera/recognition_objects", RecognitionObject, feature_callback)
    rospy.Subscriber(vehicle_name+"/gps/values", NavSatFix, gps_callback)
    rospy.Subscriber(vehicle_name+"/gyro/values", Imu, gyro_callback)
    rospy.Subscriber("control_command", ControlCommand, control_callback)
    # Publishers:
    state_pub = rospy.Publisher('state_estimate', Pose2D, queue_size=10)
    feature_pub = rospy.Publisher('features', SLAMFeature, queue_size=50)
    road_pub = rospy.Publisher('road_features', SLAMFeature, queue_size=50)
    while True:
        solver.predict(u)
        if len(feature_buffer) != 0:
            z = z.extend(feature_buffer)
            feature_buffer = []
        solver.update(z)
        z = z[0:3] # remove features
        pose = Pose2D(x=solver.x[0],y=solver.x[1],theta=solver.x[2])
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
u = np.zeros(4)
solver = EKF_solver
rospy.init_node('controller')
rospy.Subscriber("model_name", String, callback)
main()
