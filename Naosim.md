
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
RUN apt-get install -y ros-indigo-moveit-full
RUN apt-get install -y ros-indigo-ompl
RUN apt-get install -y ros-indigo-control-toolbox
RUN apt-get install -y ros-indigo-teleop-twist-keyboard
RUN apt-get -y update

# Nao packages
RUN apt-get install -y ros-indigo-nao-dcm-bringup
RUN apt-get install -y ros-indigo-naoqi-dcm-driver
RUN apt-get install -y ros-indigo-nao-robot
RUN apt-get install -y ros-indigo-nao-extras
RUN apt-get install -y ros-indigo-nao-gazebo-plugin
RUN apt-get install -y ros-indigo-driver-base
RUN apt-get install -y ros-indigo-move-base-msgs
RUN apt-get install -y ros-indigo-octomap
RUN apt-get install -y ros-indigo-octomap-msgs
RUN apt-get install -y ros-indigo-humanoid-msgs
RUN apt-get install -y ros-indigo-humanoid-nav-msgs
RUN apt-get install -y ros-indigo-camera-info-manager
RUN apt-get install -y ros-indigo-camera-info-manager-py
RUN apt-get -y update
