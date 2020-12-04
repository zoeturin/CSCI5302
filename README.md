# CSCI5302
Advanced robotics final project

USING ROS: -----------------------------------------------------------------

To install webots_ros: 

git clone https://github.com/cyberbotics/webots_ros in the src directory of a catkin workspace. 

Can instead run sudo apt install ros-melodic-webots-ros, but this seems to not be up to date (missing automobile_get_dimensions.srv). A copy of automobile_get_dimensions.srv is in simple_control/srv so you can try uncommenting "from simple_control.srv import \*" in go_forward_controller but no guarantees this will work

git clone this repo in the src file of your catkin workspace

run catkin_make (or catkin build if you prefer)

remember to source (run source devel/setup.bash in the base directory of your catkin workspace)

Open webots separately, open correct world

Set vehicle controller to "ros_automobile"

To run go_forward_controller: roslaunch simple_control main.launch

PACKAGES: --------------------------------------------------------------------

simple_control: makes car drive forward
