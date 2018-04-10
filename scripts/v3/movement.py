# !/usr/bin/env python2.7

import sys
#import naoqi
import time
#import almath
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

		# 2.0857 is leftmost radian robot can turn it's head
		# -2.0857 is rightmost radian robot can turn it's head
		# -0.6720 is uppermost radian robot can tilt it's head
		# 0.5149 is lowermost radian robot can tilt it's head
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
			self.interval = 0.5
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
			px = self.pp[1]
			py = self.pp[0]

			if score >= 0.5:
				self.interval = 0.2
				sharp = 0.4

			else:
				self.interval = 0.5
				sharp = 0.2

			if py + sharp > 0.5:
				py = 0.4 - sharp

			i = self.interval
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + sharp, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)
			self.target()

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
				self.interval = 0.3
				incline = 0.2
				sharp = 0.2

			# checks to make sure that the nao doesn't exceed it's range of movement
			if py + incline > 0.5:
				incline = 0
			if -1.4 > px:
				px = -1.4 + sharp
			if 1.4 < px:
				px = 1.4 - sharp

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
			goal = [py + incline, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + incline, px - sharp]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i)
			self.target()

		except KeyboardInterrupt:
			sys.exit()

	# causes the nao to raise it's arms and clench fists
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

	# causes the nao to look away from players face
	def idle(self):
		# nao can't look within +- [0.05, 0.2] radians of the players face or look further than +- [0.25, 0.4]
		px = 0
		py = 0
		self.interval = 0.2

		px += uniform(-0.2, 0.2)
		if px < 0:
			px -= 0.2
		else:
			px += 0.2

		py += uniform(-0.2, 0.2)
		if py < 0:
			py -= 0.05
		else:
			py += 0.05

		ty = self.pp[0] + py
		tx = self.pp[1] + px

		if ty > 0.5:
			ty = 0.5
		if ty < -0.5:
			ty = -0.5

		if tx > 1.9:
			tx = 1.9

		if tx < -1.9:
			tx = -1.9

		pos = [ty, tx]
		self.move(pos, self.ph)
		rospy.sleep(0.2)

	def target(self, pos=None):
		try:
			self.interval = 0.2

			if pos is None:
				pos = self.pp

			if pos[0] > 0.4:
				pos[0] = 0.4

			if pos[0] < -0.4:
				pos[0] = -0.4

			if pos[1] > 1.5:
				pos[1] = 1.5

			if pos[1] < -1.5:
				pos[1] = -1.5


			self.jt.joint_names = self.headJ
			self.move(pos, self.ph)
			rospy.sleep(0.2)
		except KeyboardInterrupt:
			sys.exit()
