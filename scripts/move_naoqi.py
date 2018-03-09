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
		
		# publishers for each robot body part used
		self.ph = rospy.Publisher('/nao_dcm/Head_controller/command', JointTrajectory, queue_size=100)
		self.pal = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=100)
		self.par = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=100)
		
		self.r = rospy.Rate(100)
		
		# message objects and default message intervals
		self.jt = JointTrajectory()
		self.jtp = JointTrajectoryPoint()
		self.interval = 0.5
		
		# joint names for each body part used by related message
		self.headJ = ['HeadPitch', 'HeadYaw']
		self.LArmJ = ['LElbowRoll', 'LElbowYaw', 'LShoulderPitch', 'LShoulderRoll', 'LWristYaw']
		self.RArmJ = ['RElbowRoll', 'RElbowYaw', 'RShoulderPitch', 'RShoulderRoll', 'RWristYaw']
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

	# method for making the robot nodding it's head
	def head_nod(self, mood):
		try:
			self.jt.joint_names = ['HeadPitch', 'HeadYaw']
			p = self.ph
			if sad:
				self.interval = 0.4
				goal = [0.0, 0.0]
				self.move(goal, p)
				rospy.sleep(i)
				goal[0] += 0.6
				self.move(goal, p)
				rospy.sleep(i)
				goal[1] -= 0.6
				self.move(goal, p)
				rospy.sleep(i)

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
	
	# method for making the robot shake it's head
	def head_shake(self, mood):
		try:
			self.jt.joint_names = ['HeadPitch', 'HeadYaw']
			p = self.ph
			i = self.interval
			if mood >= 0.5:
				i = 0.5
				goal = [0.4, 1.0]

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

# message on program exit
def my_hook():
	print "shutting down"


rospy.on_shutdown(my_hook)

test = Mover()
mood = 0.5

while not rospy.is_shutdown():
	try:
		test.body_reset()
		#test.head_shake(False)
		#test.head_nod(True)

	except KeyboardInterrupt:
		sys.exit()
