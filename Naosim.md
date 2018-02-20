
roslaunch nao_gazebo_plugin nao_gazebo_plugin_H25.launch
sudo ln /dev/null /dev/raw1394
naoqi --verbose --broker-ip 127.0.0.1
roslaunch nao_bringup nao_full_py.launch nao_ip:=127.0.0.1 roscore_ip:=127.0.0.1
rosservice call /body_stiffness/enable "{}"
roslaunch nao_moveit_config moveit_planner.launch




#example head command
trajectory_msgs/JointTrajectory
"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
 ['HeadPitch', 'HeadYaw']
points:
- positions: [-0.0, -2]
  velocities: []
  accelerations: []
  effort: []
