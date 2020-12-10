#!/usr/bin/env python2

import rospy
import copy
from ros_service import *
from mapping import *
from webots_ros.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from localization.msg import SLAMFeature, SLAMFeatureArray

# Flags:
enable_map_display = True

def vehicle_name_callback(name):
    global vehicle_name
    vehicle_name = name.data
    rospy.loginfo("Got vehicle name")

def state_est_callback(data):
    global pose, prev_pose, new_pose
    pose = [data.x, data.y, data.theta]
    new_pose = True

def feature_callback(data):
    global feature_pose_buffer, feature_buffer
    feature_buffer = data.feature_array
    feature_pose_buffer = []
    for feature in data.feature_array:
        pose = [feature.position.x, feature.position.z, 0] # update theta in the future?
        if feature.model == 'road':
            # road_buffer.append(pose)
            pass
        else:
            feature_pose_buffer.append(pose)

def tf_coords_for_display(coords, disp_mins, scaling=1):
    new_coords = copy.copy(coords)
    new_coords = np.array(new_coords)
    new_coords *= scaling
    new_coords -= disp_mins
    new_coords = [int(round(coord)) for coord in new_coords]
    return new_coords

def main():
    global pose, prev_pose, new_pose
    global feature_pose_buffer, feature_buffer, vehicle_name
    map_range = [[-50, 50],[-100, 100]] # [[xmin, xmax],[ymin,ymax]]
    map_mins = [map_range[0][0], map_range[1][0]]
    occMap = OccMap(map_range[0][0],map_range[0][1],map_range[1][0],map_range[1][1], 1)
    #roadMap = OccMap(map_range[0][0],map_range[0][1],map_range[1][0],map_range[1][1], 1)
    while vehicle_name is None:
        pass
    # Services:
    if enable_map_display:
        draw_pixel = service(vehicle_name+"/display/draw_pixel", display_draw_pixel)
        draw_rectangle = service(vehicle_name+"/display/draw_rectangle", display_draw_rectangle)
        draw_text = service(vehicle_name+"/display/draw_text", display_draw_text)
        draw_line = service(vehicle_name+"/display/draw_line", display_draw_line)
        set_color = service(vehicle_name+"/display/set_color", set_int)
        rospy.loginfo('Map node: GOT SERVICES')
    # Subscribers:
    rospy.Subscriber('state_estimate', Pose2D, state_est_callback)
    rospy.Subscriber('features', SLAMFeatureArray, feature_callback)
    # Wait for EKF:
    while pose is None:
        pass
    rospy.loginfo('READY TO MAP')
    while not rospy.is_shutdown():
        if feature_buffer:
            print('updating map')
            #occMap.update_map(pose, feature_pose_buffer)
            if enable_map_display:
                print('drawing')
                print(len(feature_buffer))
                for feature in feature_buffer:
                    set_color.srv(255*65536+255*256+255) # white
                    #draw_text.srv(text=feature.model,x=feature.position.x,y=feature.position.z)
                    # feature_pos = [feature.position.x, feature.position.z]
                    # feature_pos_display = tf_coords_for_display(feature_pos, map_mins)
                    # draw_pixel.srv(x1=feature_pos_display[0],y1=feature_pos_display[1])
            feature_buffer = []
            feature_pose_buffer = []
        if new_pose and enable_map_display:
            if prev_pose is None:
                prev_pose = pose
            print("drawing_line")
            set_color.srv(0*65536+255*256+0) # green
            prev_pose_display = tf_coords_for_display(prev_pose[0:2], map_mins)
            new_pose_display = tf_coords_for_display(pose[0:2], map_mins)
            draw_line.srv(x1=prev_pose_display[0],y1=prev_pose_display[1],x2=new_pose_display[0],y2=new_pose_display[1])
            new_pose = False
            prev_pose = pose

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
