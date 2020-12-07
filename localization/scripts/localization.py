import rospy
from EKF.py import *
from webots_ros.msg import RecognitionObject
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D
'''
TODO:
- add publisher for features (model, position)

'''
def vehicle_name_callback(name):
    global vehicle_name
    vehicle_name = name.data
    rospy.loginfo("Got vehicle name")

def feature_callback(feature_obj):
    feature_pos = feature_obj.position
    state = np.array([feature_pos.x,feature_pos.z])+solver.x_hat[0:2]
    feature = feature(feature_obj.model, state)
    feature_buffer.append(feature)
    rospy.loginfo("Got feature")

def gps_callback(data):
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
    th_dot = data.angular_velocity.y
    cov = data.angular_velocity_covariance
    solver.gyro_cov = cov[4]
    pass

def main():
    while vehicle_name is None:
        pass
    # Subscribers:
    rospy.Subscriber(vehicle_name+"/front_camera/recognition_objects", RecognitionObject, feature_callback)
    rospy.Subscriber(vehicle_name+"/gps/values", NavSatFix, gps_callback)
    rospy.Subscriber(vehicle_name+"/gyro/values", Imu, gyro_callback)
    # Publishers:
    state_pub = rospy.Publisher('state_estimate', Pose2D, queue_size=10)
    feature_pub =
    while True:
        solver.predict(u)
        if len(feature_buffer) != 0:
            z = z.extend(feature_buffer)
            feature_buffer = []
        solver.update(z)
        z = z[0:3] # remove features
        pose = Pose2D(x=solver.x[0],y=solver.x[1],theta=solver.x[2])
        state_pub.publish(pose)

vehicle_name = None
feature_buffer = []
z = np.zeros(3)
solver = EKF_solver
rospy.init_node('controller', anonymous=True)
rospy.Subscriber("model_name", String, callback)
main()
