# !/usr/bin/env python2.7

import sys
import naoqi
from naoqi import ALProxy
import time
import almath
import numpy
import rospy
import roslib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from random import uniform, randint

class Mover:
	def __init__(self):
		rospy.init_node('hang_nao', anonymous=True)
		# publishers for each robot body part used
		self.ph = rospy.Publisher('/nao_dcm/Head_controller/command', JointTrajectory, queue_size=1)
		self.pal = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
		self.par = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
		self.phl = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=1)
		self.phr = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=1)

		# subscribers for robot sensors
		#rospy.Subscriber('/nao_dcm/Head_controller/state', JointTrajectoryControllerState, self.head_pos)
		self.r = rospy.Rate(10)

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
		self.pp = [0, 0]
		self.body_reset()
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
	def head_nod(self, score):
		try:
			self.jt.joint_names = self.headJ
			p = self.ph
			if score >= 0.5:
				self.interval = 0.3
				sharp = 0.6

			else:
				self.interval = 0.2
				sharp = 0.2

			i = self.interval
			px = self.pp[1]
			py = self.pp[0]
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + sharp, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)

		except KeyboardInterrupt:
			sys.exit()

	# method for making the robot shake it's head
	def head_shake(self, score):
		try:
			self.jt.joint_names = self.headJ
			p = self.ph
			px = self.pp[1]
			py = self.pp[0]

			if score >= 0.5:
				self.interval = 0.2
				incline = 0
				sharp = 0.3

			else:
				self.interval = 0.4
				incline = 0.4
				sharp = 0.6

			i = self.interval
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + incline, px - sharp]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + incline, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + incline, px + sharp]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + incline, px - sharp]
			self.move(goal, p)  #
			rospy.sleep(i)
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)

		except KeyboardInterrupt:
			sys.exit()

	#
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

	def idle(self):
		px = uniform(-0.4, 0.4)
		if px < 0:
			px -= 0.2
		else:
			px += 0.2

		py = uniform(-0.2, 0.2)
		if py < 0:
			py -= 0.1
		else:
			py += 0.1

		pos = [self.pp[0]+py, self.pp[1] + px]
		self.move(pos, self.ph)

	def target(self):
		try:
			#cwl = 2.0857  #leftmost radian robot can turn it's head
			#cwr = -2.0857  #rightmost radian robot can turn it's head
			#chu = -0.6720  #uppermost radian robot can tilt it's head
			#chd = 0.5149  #lowermost radian robot can tilt it's head
			#vpw = 1.0630/2   #vertical field of view for the robot halved
			#vph = 0.8308/2   #horizontal field of view for the robot halved

			self.jt.joint_names = self.headJ
			pos = self.pp
			self.move(pos, self.ph)

		except KeyboardInterrupt:
			sys.exit()