#commands
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

#example arm command
header: 
  seq: 217
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
joint_names: ['RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll', 'RWristYaw']
points: 
  - 
    positions: [0.03, 0.0, -0.0, -0.0, 0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 1
      nsecs:         0
---


#installation

# Extra packages
sudo apt-get -y update
sudo apt-get install -y ros-indigo-moveit-full ros-indigo-ompl ros-indigo-control-toolbox ros-indigo-teleop-twist-keyboard


# Nao packages
sudo apt-get -y update
sudo apt-get install -y ros-indigo-nao-dcm-bringup ros-indigo-naoqi-dcm-driver ros-indigo-nao-robot ros-indigo-nao-extras ros-indigo-nao-gazebo-plugin ros-indigo-driver-base ros-indigo-move-base-msgs ros-indigo-octomap ros-indigo-octomap-msgs ros-indigo-humanoid-msgs ros-indigo-humanoid-nav-msgs ros-indigo-camera-info-manager ros-indigo-camera-info-manager-py ros-indigo-nao-meshes

git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
git clone https://github.com/pal-robotics/pal_msgs.git
git clone https://github.com/pal-robotics/pal_gazebo_plugins.git
catkin_make
source devel/setup.bash
