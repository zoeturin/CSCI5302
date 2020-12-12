# CSCI5302
Advanced robotics final project

USING ROS: -----------------------------------------------------------------

To install webots_ros: 

git clone https://github.com/cyberbotics/webots_ros in the src directory of a catkin workspace. 

Can instead run sudo apt install ros-melodic-webots-ros, but this seems to not be up to date (missing automobile_get_dimensions.srv and perhaps some launch files). A copy of automobile_get_dimensions.srv is in simple_control/srv to solve this issue though missing/different launch files in webots_ros may still be an issue.

git clone this repo in the src file of your catkin workspace

run catkin_make (or catkin build if you prefer)

remember to source (run source devel/setup.bash in the base directory of your catkin workspace)

Open webots separately (FROM THE TERMINAL OR A LAUNCH FILE), open correct world

WORLD/WEBOTS SETUP: ----------------------------------------------------------

Set vehicle controller to "ros_automobile"

To display the vehicle path map in webots add a display object to the vehicle with width equal to the width (x dimension) of the map and height equal to the height (z dimension) of the map. 

Set the controller args in the .wbt file to include "--clock" "--synchronize" and "--use-sim-time"

The accuracy of the GPS should be set to a non-zero value.

You may have to set the WEBOTS_HOME environment variable as described in the webots documentation

The basic timestep for the world should be 5 ms.

TEST CONTROLLER AND EKF: ----------------------------------------------------

set the path in loc_launch.launch (in the localization package) to the desired world file (this file should be configured as described above). The world file we used for testing is in controller/worlds

run roslaunch localization loc_launch.launch

This will run the controller and the EKF. EKF state estimates are published to the topic /state_estimates. Known features (their poses, model names and an id number) are published to /features. Feature states are also recorded in OccMap in mapping_node.py, where the origin of a feature is represented by an occupied cell. 

TEST PARALLEL PARKING: ------------------------------------------------------

[Leo pls add instructions here]

PACKAGES: --------------------------------------------------------------------

simple_control: makes car drive forward, can use as a template for other webots_ros controllers as it contains necessary subscribers and service calls 

localization: EKF SLAM and map display nodes

controller: contains controller node which will navigate track based on lane lines
