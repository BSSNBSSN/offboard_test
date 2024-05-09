# offboard_test


```
roslaunch mavros px4.launch
roslaunch vrpn_client_ros sample.launch server:=192.168.3.4
roslaunch offboard bringup.launch 
rosrun offboard figure_circle_node
rostopic echo /mavros/local_position/odom
rosbag record /mavros/local_position/odom
```