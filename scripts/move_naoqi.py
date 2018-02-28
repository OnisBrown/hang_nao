#!/usr/bin/env python2.7

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
		self.ph = rospy.Publisher('/nao_dcm/Head_controller/command', JointTrajectory, queue_size=100)
		self.pal = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=100)
		self.par = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=100)
		self.r = rospy.Rate(100)
		self.jt = JointTrajectory()
		self.jtp = JointTrajectoryPoint()
		self.interval = 0.5
		self.headJ = ['HeadPitch', 'HeadYaw']
		self.LArmJ = ['LElbowRoll', 'LElbowYaw', 'LShoulderPitch', 'LShoulderRoll', 'LWristYaw']
		self.RArmJ = ['RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll', 'RWristYaw']
		print "Nao mover node ready"

	def pub(self, p):
		p.publish(self.jt)

	def move(self, goal, p):
		try:
			self.move_setup()
			self.jtp.positions = goal
			self.jtp.time_from_start = rospy.Duration(self.interval)
			self.jt.points.append(self.jtp)
			self.pub(p)

		except KeyboardInterrupt:
			sys.exit()

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

	def body_reset(self):
		try:
			self.move_setup()
			self.jt.joint_names = self.headJ
			goal = [0.0, 0.0]
			self.move(goal, self.ph)
			self.move_setup()
			self.jt.joint_names = self.LArmJ
			goal = [0.0, 0.0, 0.0, 0.0, 0.0]
			self.move(goal, self.pal)
			self.jt.joint_names = self.RArmJ
			goal = [0.0, 0.0, 0.0, 0.0, 0.0]
			self.move(goal, self.par)

		except KeyboardInterrupt:
			sys.exit()


	def head_nod(self, sad):
		try:
			self.jt.joint_names = ['HeadPitch', 'HeadYaw']
			p = self.ph
			if sad:
				self.interval = 0.4
				goal = [0.0, 0.0]
				self.move(goal, p)
				goal[0] += 0.6
				self.move(goal, p)
				goal[1] -= 0.6
				self.move(goal, p)

			else:
				self.interval = 0.5
				goal = [0.0, 0.0]
				self.move(goal, p)
				goal[0] += 1
				self.move(goal, p)
				goal[0] -= 1
				self.move(goal, p)

		except KeyboardInterrupt:
			sys.exit()

	def head_shake(self, sad):
		try:
			self.jt.joint_names = ['HeadPitch', 'HeadYaw']
			p = self.ph
			i = self.interval
			if sad:
				i = 0.5
				goal = [0.4, 1.0]
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] -= 1
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] -= 1
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] += 1
				self.move(goal, p)
				rospy.sleep(i)

			else:
				i = 0.2
				goal = [0.0, 1.0]
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] -= 1
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] -= 1
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] += 1
				self.move(goal, p)
				rospy.sleep(i)

		except KeyboardInterrupt:
			sys.exit()


def my_hook():
	print "shutting down"


rospy.on_shutdown(my_hook)

test = Mover()

while not rospy.is_shutdown():
	try:
		test.body_reset()
		#test.head_shake(False)
		#test.head_nod(True)

	except KeyboardInterrupt:
		sys.exit()
