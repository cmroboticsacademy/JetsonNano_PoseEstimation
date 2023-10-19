echo "**** Starting... ****"
roslaunch ros_deep_learning video_source.ros1.launch &
sleep 5
echo "**** running jetbot_control ****"
rosrun jetbot_ros jetbot_control.py &
sleep 2
echo "**** running navigation_dev ****"
roslaunch navigation_dev navigation3.launch &
