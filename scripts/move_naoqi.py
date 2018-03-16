# !/usr/bin/env python2.7

import sys
import naoqi
from naoqi import ALProxy
import time
import almath
import rospy
import roslib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Mover:
	def __init__(self):
		rospy.init_node('mover', anonymous=True)

		# publishers for each robot body part used
		self.ph = rospy.Publisher('/nao_dcm/Head_controller/command', JointTrajectory, queue_size=100)
		self.pal = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=100)
		self.par = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=100)
		self.phl = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=100)
		self.phr = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=100)
		self.r = rospy.Rate(100)

		# message objects and default message intervals
		self.jt = JointTrajectory()
		self.jtp = JointTrajectoryPoint()
		self.interval = 0.5

		# joint names for each body part used by related message
		self.headJ = ['HeadPitch', 'HeadYaw']
		self.LArmJ = ['LElbowRoll', 'LElbowYaw', 'LShoulderPitch', 'LShoulderRoll', 'LWristYaw']
		self.RArmJ = ['RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll', 'RWristYaw']
		self.RHJ = ['RHand']
		self.LHJ = ['LHand']
		print "Nao mover node ready"

	# publisher method that accepts a publisher object and uses it
	def pub(self, p):
		p.publish(self.jt)

	# movement method for creating movement messages
	def move(self, goal, p):
		try:
			self.move_setup()
			self.jtp.positions = goal
			self.jtp.time_from_start = rospy.Duration(self.interval)
			self.jt.points.append(self.jtp)
			self.pub(p)

		except KeyboardInterrupt:
			sys.exit()

	# method for setting up move messages by setting all movement parameters to null
	def move_setup(self):
		try:
			self.jt.points = []
			self.jtp.positions = []
			self.jtp.velocities = []
			self.jtp.accelerations = []
			self.jtp.effort = []
			self.jtp.time_from_start = rospy.Duration()

		except KeyboardInterrupt:
			sys.exit()

	# method for returning the robot to a neutral position
	def body_reset(self):
		try:
			self.jt.joint_names = self.headJ
			goal = [0.0, 0.0]
			self.move(goal, self.ph)
			self.jt.joint_names = self.LHJ
			goal = [0.0]
			self.move(goal, self.phl)
			self.jt.joint_names = self.RHJ
			goal = [0.0]
			self.move(goal, self.phr)
			self.jt.joint_names = self.LArmJ
			goal = [-0.3, -1.5, 1.5, 0.2, 0.0]
			self.move(goal, self.pal)
			self.jt.joint_names = self.RArmJ
			goal = [0.3, 1.5, 1.5, -0.2, 0.0]
			self.move(goal, self.par)

		except KeyboardInterrupt:
			sys.exit()

	# method for making the robot nodding it's head
	def head_nod(self, mood):
		try:
			self.jt.joint_names = self.headJ
			p = self.ph

			if mood >= 0.5:
				self.interval = 0.3
				sharp = 0.6

			else:
				self.interval = 0.2
				sharp = 0.2

			i = self.interval
			goal = [0.0, 0.0]
			self.move(goal, p)
			rospy.sleep(i)
			goal[0] += sharp
			self.move(goal, p)
			rospy.sleep(i)
			goal[0] -= sharp
			self.move(goal, p)
			rospy.sleep(i)

		except KeyboardInterrupt:
			sys.exit()

	# method for making the robot shake it's head
	def head_shake(self, mood):
		try:
			self.jt.joint_names = self.headJ
			p = self.ph

			if mood >= 0.5:
				self.interval = 0.2
				goal = [0.0, 0.5]
				sharp = 0.5

			else:
				self.interval = 0.4
				goal = [0.4, 1.0]
				sharp = 1

			i = self.interval
			self.move(goal, p)
			rospy.sleep(i)
			goal[1] -= sharp
			self.move(goal, p)
			rospy.sleep(i)
			goal[1] -= sharp
			self.move(goal, p)
			rospy.sleep(i)
			goal = [0.0, 0.0]
			self.move(goal, p)
			rospy.sleep(i)

		except KeyboardInterrupt:
			sys.exit()

	def cheer(self):
		try:
			i = self.interval
			self.jt.joint_names = self.LHJ
			goal = [1.0]
			self.move(goal, self.phl)
			self.jt.joint_names = self.RHJ
			goal = [1.0]
			self.move(goal, self.phr)
			self.jt.joint_names = self.LArmJ
			goal = [-1.0, -0.5, -1.5, 1.0, 0.0]
			self.move(goal, self.pal)
			self.jt.joint_names = self.RArmJ
			goal = [1.0, 0.5, -1.5, -1.0, 0.0]
			self.move(goal, self.par)
			rospy.sleep(i)
			self.jt.joint_names = self.LArmJ
			goal = [-1.0, -0.5, -1.5, 0.6, 0.0]
			self.move(goal, self.pal)
			self.jt.joint_names = self.RArmJ
			goal = [1.0, 0.5, -1.5, -0.6, 0.0]
			self.move(goal, self.par)
			rospy.sleep(i)
			self.jt.joint_names = self.LArmJ
			goal = [-1.0, -0.5, -1.5, 1.0, 0.0]
			self.move(goal, self.pal)
			self.jt.joint_names = self.RArmJ
			goal = [1.0, 0.5, -1.5, -1.0, 0.0]
			self.move(goal, self.par)

		except KeyboardInterrupt:
			sys.exit()