# !/usr/bin/env python2.7

import sys
import naoqi
from naoqi import ALProxy
import time
import almath
import numpy
import rospy
import roslib
import cv2, cv_bridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import Image

from random import uniform, randint

class Mover:
	def __init__(self):
		rospy.init_node('hang_nao', anonymous=True)

		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)

		# publishers for each robot body part used
		self.ph = rospy.Publisher('/nao_dcm/Head_controller/command', JointTrajectory, queue_size=100)
		self.pal = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=100)
		self.par = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=100)
		self.phl = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=100)
		self.phr = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=100)

		# subscribers for robot sensors
		#rospy.Subscriber('/nao_dcm/Head_controller/state', JointTrajectoryControllerState, self.head_pos)
		rospy.Subscriber('/nao_robot/camera/top/image_raw', Image, self.head_view)
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

		self.change = False
		self.current = [0, 0]
		self.score = 0.0
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
			self.target()

		except KeyboardInterrupt:
			sys.exit()

	# method for making the robot nodding it's head
	def head_nod(self):
		try:
			self.jt.joint_names = self.headJ
			p = self.ph
			if self.score >= 0.5:
				self.interval = 0.3
				sharp = 0.6

			else:
				self.interval = 0.2
				sharp = 0.2

			i = self.interval
			px = self.pp[1]
			py = self.pp[0]
			goal = [py + sharp, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i*3)
			self.target()

		except KeyboardInterrupt:
			sys.exit()

	# method for making the robot shake it's head
	def head_shake(self):
		try:
			self.jt.joint_names = self.headJ
			p = self.ph
			px = self.pp[1]
			py = self.pp[0]

			if self.score >= 0.5:
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
			self.move(goal, p)#
			rospy.sleep(i)
			goal = [py + incline, px]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py + incline, px + sharp]
			self.move(goal, p)
			rospy.sleep(i)
			goal = [py, px]
			self.move(goal, p)
			rospy.sleep(i*3)
			self.target()

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

	#def head_pos(self, state):
	#	self.current = list(state.desired.positions)

	def head_view(self, img):
		image = self.bridge.imgmsg_to_cv2(img,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		cv2.imshow("window", image)
		cv2.waitKey(3)

	def target(self):
		try:
			#cwl = 2.0857  #leftmost radian robot can turn it's head
			#cwr = -2.0857  #rightmost radian robot can turn it's head
			#chu = -0.6720  #uppermost radian robot can tilt it's head
			#chd = 0.5149  #lowermost radian robot can tilt it's head
			#vpw = 1.0630/2   #vertical field of view for the robot halved
			#vph = 0.8308/2   #horizontal field of view for the robot halved
			self.change = False
			lt = uniform(0.5, 1.5) + self.score
			bt = uniform(1.5, 2.1) - self.score
			px = uniform(-1.8, 1.8)
			py = uniform(-0.5, 0.4)

			if self.change:
				self.jt.joint_names = self.headJ

				pos = self.pp
				self.move(pos, self.ph)

				rospy.sleep(lt)


				pos = [py, px]
				self.move(pos, self.ph)
				rospy.sleep(bt)

				pos = self.pp
				self.move(pos, self.ph)
				rospy.sleep(1)
				self.change = True


		except KeyboardInterrupt:
			sys.exit()
