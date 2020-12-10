#!/usr/bin/env python2

import rospy
from ros_service import *
from mapping import *
from webots_ros.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from localization.msg import SLAMFeature

# Flags:
enable_map_display = True

def vehicle_name_callback(name):
    global vehicle_name
    vehicle_name = name.data
    rospy.loginfo("Got vehicle name")

def state_est_callback(data):
    global pose, prev_pose, new_pose
    prev_pose = pose
    pose = [data.x, data.y, data.theta]
    new_pose = True

def feature_callback(data):
    global feature_pose_buffer, feature_buffer
    pose = [data.position.x, data.position.z, 0] # update theta in the future?
    feature_pose_buffer.append(pose)
    feature_buffer.append(data)
    # if data.model == 'road':
    #     road_buffer.append(pose)
    # else:
    #     feature_pose_buffer.append(pose)

def main():
    global pose, prev_pose, new_pose
    global feature_pose_buffer, road_buffer, vehicle_name
    map_range = [[-50, 50],[-100, 100]] # [[xmin, xmax],[ymin,ymax]]
    occMap = OccMap(map_range[0][0],map_range[0][1],map_range[1][0],map_range[1][1], 1)
    roadMap = OccMap(map_range[0][0],map_range[0][1],map_range[1][0],map_range[1][1], 1)
    while vehicle_name is None:
        pass
    # Services:
    if enable_map_display:
        draw_pixel = service(vehicle_name+"/display/draw_pixel", display_draw_pixel)
        draw_rectangle = service(vehicle_name+"/display/draw_rectangle", display_draw_rectangle)
        draw_text = service(vehicle_name+"/display/draw_text", display_draw_text)
        draw_line = service(vehicle_name+"/display/draw_line", display_draw_line)
        set_color = service(vehicle_name+"/display/set_color", set_int)
        rospy.logwarn('GOT SERVICES')
    # Subscribers:
    rospy.Subscriber('state_estimate', Pose2D, state_est_callback)
    rospy.Subscriber('features', SLAMFeature, feature_callback)
    # Wait for EKF:
    while pose is None or prev_pose is None:
        pass
    rospy.logwarn('READY TO MAP')
    while not rospy.is_shutdown():
        if feature_pose_buffer:
            rospy.loginfo('updating map')
            occMap.update_map(pose, feature_pose_buffer)
            if enable_map_display:
                rospy.logwarn('drawing')
                for feature in feature_buffer:
                    draw_text.srv(text=feature.model,x=feature.position.x,y=feature.position.z)
                    draw_line.srv(x1=prev_pose[0],y1=prev_pose[1],x2=pose[0],y2=pose[1])
            feature_pose_buffer = []
        if new_pose and enable_map_display:
            x1 = prev_pose[0]-map_range[0][0]
            y1 = prev_pose[1]-map_range[1][0]
            x2 = pose[0]-map_range[0][0]
            y2 = pose[1]-map_range[1][0]
            draw_line.srv(x1=x1,y1=y1,x2=x2,y2=y2)
            new_pose = False
        # if road_buffer:
        #     roadMap.update_map(pose, road_buffer)
        #     road_buffer = []

pose = None
vehicle_name = None
prev_pose = None
new_pose = False
feature_pose_buffer = []
feature_buffer = []
road_buffer = []
rospy.init_node("map")
rospy.Subscriber("model_name", String, vehicle_name_callback)
main()
