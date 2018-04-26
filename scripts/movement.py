# !/usr/bin/env python2.7

import sys
#import naoqi
import time
#import almath
import math
import rospy
import roslib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from random import uniform, randint
from naoqi import ALProxy


class Mover:
	def __init__(self):
		rospy.init_node('hang_nao', anonymous=True)

		#publishers for the real robot
		self.pj = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size = 1)
		#naoqi motion proxy
		IP = "192.168.1.3"
		self.motionProxy = ALProxy("ALMotion", IP, 9559)

		# subscribers for robot sensors
		rospy.Subscriber('/joint_states', JointState, self.head_update)
		self.r = rospy.Rate(10)

		# message objects and default speed intervals
		self.ja = JointAnglesWithSpeed()
		self.interval = 0.5 # time between actions

		# 2.0857 is leftmost radian robot can turn it's head
		# -2.0857 is rightmost radian robot can turn it's head
		# -0.6720 is uppermost radian robot can tilt it's head
		# 0.5149 is lowermost radian robot can tilt it's head
		self.limitH = 2
		self.limitV = 0.4
		self.HY = 0.0
		self.HX = 0.0
		self.speed = 0.1 # global move speed for head joints, other joints are specified
		self.headJ = ['HeadPitch', 'HeadYaw']
		self.LArmJ = ['LElbowRoll', 'LElbowYaw', 'LShoulderPitch', 'LShoulderRoll', 'LWristYaw']
		self.RArmJ = ['RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll', 'RWristYaw']
		self.RHJ = ['RHand']
		self.LHJ = ['LHand']
		self.hips = ['LHipYawPitch', 'RHipPitch', 'LHipPitch'] #both hipYaws are on the same motor
		self.pp = [0, 0]


		# set the joint stiffness for head, arms and waist
		names = ['LHipYawPitch', 'RHipPitch', 'LHipPitch', 'RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll',
				 'RWristYaw', 'LElbowRoll', 'LElbowYaw', 'LShoulderPitch', 'LShoulderRoll', 'LWristYaw', 'HeadPitch',
				 'HeadYaw']
		stiffnesses = 1.0
		self.motionProxy.setStiffnesses(names, stiffnesses)

		self.body_reset()
		print "Nao mover node ready"

	def __del__(self):
		print "assuming resting position in 3 seconds"
		time.sleep(3)
		self.motionProxy.rest()

	def head_update(self, pos):
		self.HY = pos.position[1]
		self.HX = pos.position[0]


	def range(self, goal):
		a = abs(goal[0] - self.HY)
		b = abs(goal[1] - self.HX)
		return math.sqrt(math.pow(a, 2) + math.pow(b, 2))


	# publisher method that accepts a publisher object and uses it
	def pub(self):
		self.pj.publish(self.ja)

	# movement method for creating movement messages
	def move(self, goal):
		try:
			self.move_setup()
			self.ja.joint_angles = goal
			self.ja.speed = self.speed
			self.pub()

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()

	# method for setting up move messages by setting all movement parameters to null
	def move_setup(self):
		try:
			self.ja.joint_angles = []
			self.ja.speed = 0.0

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()

	# method for returning the robot to a neutral position
	def body_reset(self):
		try:
			self.interval = 0.5
			self.speed = 0.5
			self.ja.joint_names = self.LHJ + self.RHJ + self.LArmJ + self.RArmJ
			goal = [0.0] + [0.0] + [-0.3, -1.5, 1.7, 0.2, 0.0] + [0.3, 1.5, 1.7, -0.2, 0.0]
			self.move(goal)
			rospy.sleep(self.interval)

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()

	# method for making the robot nodding it's head
	def head_nod(self, score):
		try:
			self.ja.joint_names = self.headJ
			px = self.pp[1]
			py = self.pp[0]

			if score >= 0.5:
				sharp = 0.5

			else:
				sharp = 0.3

			if py + sharp > self.limitV:
				py = self.limitV - sharp

			# pitch head to player
			goal = [py, px]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# pitch head forwards
			goal = [py + sharp, px]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# pitch head back to original
			goal = [py, px]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()

	# method for making the robot shake it's head
	def head_shake(self, score):
		try:
			self.ja.joint_names = self.headJ
			px = self.pp[1]
			py = self.pp[0]

			if score >= 0.5:
				sharp = 0.5

			else:
				sharp = 0.7

			# checks to make sure that the nao doesn't exceed it's range of movement

			if -self.limitH > px:
				sharp = -1 * self.limitH + sharp
			elif self.limitH < px:
				sharp = self.limit - sharp

			# incline head if necessary
			goal = [py, px]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# turn to the right
			goal = [py, px - sharp]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# turn back to original angle
			goal = [py, px]
			self.interval = 0.4
			rospy.sleep(self.interval)

			# turn to the left
			goal = [py, px + sharp]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# turn back to original angle
			goal = [py, px]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# turn to the right
			goal = [py, px - sharp]
			self.interval = 0.4
			self.move(goal)
			rospy.sleep(self.interval)

			# turn back to original angle
			goal = [py, px]
			self.interval = 0.2
			self.move(goal)
			rospy.sleep(self.interval)

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()

	# causes the nao to raise it's arms and clench fists
	def cheer(self):
		try:
			self.interval = 0.5
			self.speed = 0.5
			i = self.interval
			self.ja.joint_names = self.LHJ + self.RHJ
			goal = [1.0] + [1.0]
			self.move(goal)
			rospy.sleep(i)
			self.ja.joint_names = self.LArmJ + self.RArmJ
			goal = [-1.0, -0.5, -1.5, 1.0, 0.0] + [1.0, 0.5, -1.5, -1.0, 0.0]
			self.move(goal)
			rospy.sleep(i)
			self.ja.joint_names = self.LArmJ + self.RArmJ
			goal = [-1.0, -0.5, -1.5, 0.6, 0.0] + [1.0, 0.5, -1.5, -0.6, 0.0]
			self.move(goal)
			rospy.sleep(i)
			self.ja.joint_names = self.LArmJ + self.RArmJ
			goal = [-1.0, -0.5, -1.5, 1.0, 0.0] + [1.0, 0.5, -1.5, -1.0, 0.0]
			self.move(goal)

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()

	# causes the nao to look away from players face
	def idle(self):
		# nao can't look within +- [0.2, 0.3] radians of the players face or look further than +- [0.3, 0.5]
		px = 0
		py = 0

		px += uniform(-0.2, 0.2)
		if px < 0:
			px -= 0.3
		else:
			px += 0.3

		py += uniform(-0.1, 0.1)
		if py < 0:
			py -= 0.2
		else:
			py += 0.2

		ty = self.pp[0] + py
		tx = self.pp[1] + px

		if abs(ty) > self.limitV:
			if ty > 0:
				ty = self.limitV
			else:
				ty = -1 *self.limitV

		if abs(tx) > self.limitH:
			if tx > 0:
				tx = self.limitH
			else:
				tx = -1 *self.limitH

		pos = [ty, tx]
		self.interval = 0.3
		self.speed = 0.05
		self.move(pos)
		time.sleep(self.interval)

	def target(self, pos=None):
		try:
			if pos is None:
				pos = self.pp

			if abs(pos[0]) > self.limitV:
				if pos[0] > 0:
					pos[0] = self.limitV

				else:
					pos[0] = -1 *self.limitV

			if abs(pos[1]) > self.limitH:
				if pos[1] > 0:
					pos[1] = self.limitH

				else:
					pos[1] = -1 *self.limitH

			self.ja.joint_names = self.headJ
			self.speed = 0.1
			self.move(pos)

		except KeyboardInterrupt:
			self.body_reset()
			sys.exit()
