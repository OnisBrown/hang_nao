import sys
import naoqi
from naoqi import ALProxy
import time
import almath
import rospy
import roslib
from movement import Mover
import game
from hang_nao.msg import GameState, NewTurn
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from control_msgs.msg import JointTrajectoryControllerState
from threading import Timer, Thread
from random import uniform, randint
import numpy

class Decisions:
	def __init__(self):
		self.score = 0.7
		self.cp = 0
		self.bgp = True
		self.change = True
		self.NM = Mover()
		self.NG = game.HangMan()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("rgb", 1)
		self.face_cascade = cv2.CascadeClassifier('haarcasade_frontalface_default.xml')
		rospy.Subscriber('/game/GameState', GameState, self.answer)
		rospy.Subscriber('/game/NewTurn', NewTurn, self.update_turn)
		rospy.Subscriber('/nao_robot/camera/top/image_raw', Image, self.head_view)
		rospy.Subscriber('/nao_dcm/Head_controller/state', JointTrajectoryControllerState, self.head_update)
		self.tracking = False
		self.idle = False
		self.HY = 0.0
		self.HX = 0.0
		self.NM.body_reset()
		self.NG.game_start()

	def head_update(self, pos):
		self.HY = pos.actual.positions[0]
		self.HX = pos.actual.positions[1]

	def yes(self):
		if self.score < 0.8:
			self.NM.head_nod(self.score)
		else:
			self.NM.head_nod(self.score)
			self.NM.cheer()
			self.NM.body_reset()

	def no(self):
		self.NM.head_shake(self.score)


	def victory(self):
		self.NM.cheer()
		self.NM.cheer()
		self.NM.body_reset()

	def defeat(self):
		self.NM.head_shake(0)
		self.NM.head_shake(0)

	def answer(self, response):
		self.trace()
		self.change = False
		self.idle = True
		if response.turn > 0:
			if bool(response.win):
				self.victory()
			else:
				if response.verify == 1:
					self.yes()
				if response.verify == 0:
					self.no()
		else:
			self.defeat()

	def head_view(self, img):
		# 1280x960 resolution
		# 60.9d ~ 1.062906r HFOV | 47.6d ~ 0.8307767r VFOV
		# 0.00083039531r per horizontal pixel | 0.00086539239 per vertical pixel
		unitX = 0.00083039531
		unitY = 0.00086539239
		diff = 99999999999
		image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = self.face_cascade.detectMultiScale(gray, 1.3, 3)
		for (x, y, w, h) in faces:
			if x > 479:
				temp = x - 479
				if temp < diff:
					diff = temp
					FposX = self.HX + x * unitX

			else:
				temp = 479 - x
				if temp < diff:
					diff = temp
					FposX = self.HX - x*unitX

			if y < 639:
				temp = 639 - y
				if temp < diff:
					diff = temp
					FposY = self.HY - y*unitY
			else:
				if temp < diff:
					diff = temp
					FposY = self.HY + y * unitY

			self.NM.pp = [FposY, FposX]

		if self.tracking:
			self.NM.target()

		cv2.imshow("rgb", image)
		cv2.waitKey(3)

	def trace(self):
		self.tracking = True
		self.idle = False
		rospy.sleep(0.5)
		lt = uniform(2.5, 3.5) + self.score  # maintains gaze for random time based on mutual gaze data
		ltt = Timer(lt, self.look_away)
		ltt.start()


	def look(self, pos):
		self.NM.target(pos)
		lt = uniform(1.5, 2) - self.score  # maintains gaze for random time based on mutual gaze data
		ltt = Timer(lt, self.trace)
		ltt.start()

	def look_away(self):
		self.tracking = False
		if self.idle:
			self.trace()
			return
		else:
			self.idle = True
		temp = [0]
		for i in self.NG.pl: # goes through player list picking out the best guesser(s)
			if i.cg > 0:
				if i.cg == temp[0]:
					temp.append(i.pos)
				if i.cg > temp[0]:
					temp = temp[:1]
					temp[0] = i.pos

		if self.change:
			if self.bgp:
				self.bgp = False
				if temp[0] != 0: # if the temp list is unchanged then the
					if len(temp) == 1:
						bestp = temp[0]
					else:
						bestp = temp[randint(0, 1)]

					if bestp != self.NM.pp: # if the bot decides that the current player is the best then code moves on
						bt = uniform(1.5, 2.0) - self.score  # time before nao looks somewhere else
						btt = Timer(bt, self.look, args=(bestp,))
						btt.start()

			else:
				self.NM.idle()
				bt = uniform(1.5, 2.0) - self.score  # time before nao looks somewhere else
				btt = Timer(bt, self.trace)
				btt.start()


	def update_turn(self, newturn):
		self.NM.body_reset()
		self.bgp = True
		self.idle = False
		self.cp = newturn.pt
		self.change = True
		player = self.NG.pl[self.cp]
		self.NM.pp = player.pos
		self.score = player.score
		self.trace()

def my_hook():
	print "shutting down"

rospy.on_shutdown(my_hook)
start = Decisions()