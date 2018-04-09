#!/usr/bin/env python2.7
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
import cv2, cv_bridge
from sensor_msgs.msg import Image
from threading import Timer
from random import uniform, randint
import numpy

class Decisions:
	def __init__(self):
		self.score = 0.7
		self.cp = 0
		self.bgp = True
		self.change = False
		self.NM = Mover()
		self.NG = game.HangMan()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("rgb", 1)
		self.face_cascade = cv2.CascadeClassifier('haarcasade_frontalface_default.xml')
		rospy.Subscriber('/game/GameState', GameState, self.answer)
		rospy.Subscriber('/game/NewTurn', NewTurn, self.update_turn)
		rospy.Subscriber('/nao_robot/camera/top/image_raw', Image, self.head_view)
		self.trace = True
		self.HY = 0.0
		self.HX = 0.0
		self.NM.body_reset()
		self.NG.game_start()

	def head_view(self, img):
		# 1280x960 resolution
		# 60.9d ~ 1.062906r HFOV | 47.6d ~ 0.8307767r VFOV
		# 0.00083039531r per horizontal pixel | 0.00086539239 per vertical pixel
		unitX = 0.00083039531
		unitY = 0.00086539239
		image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

		cv2.imshow("rgb", image)
		cv2.waitKey(3)

	def yes(self):
		if self.score < 0.8:
			self.NM.head_nod(self.score)
		else:
			self.NM.head_nod(self.score)
			self.NM.cheer()
			self.NM.body_reset()
		return

	def look(self, pos=None):
		if pos is None:
			self.NM.target()
			lt = uniform(2.5, 3.5) + self.score  # maintains gaze for random time based on mutual gaze data
			#print "attention: " + str(lt)
			ltt = Timer(lt, self.look_away)
			ltt.start()
		else:
			self.NM.target(pos)
			bt = uniform(1.5, 2.0) - self.score  # maintains gaze for random time based on mutual gaze data
			#print "away: " + str(bt)
			btt = Timer(bt, self.look)
			btt.start()
		return

	def look_away(self):
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
						return

			self.NM.idle()
			bt = uniform(1.5, 2.0) - self.score  # time before nao looks somewhere else
			btt = Timer(bt, self.look)
			btt.start()
		return

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
		self.change = False
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

	def update_turn(self, newturn):
		rospy.sleep(0.5)

		self.NM.body_reset()
		self.bgp = True
		self.cp = newturn.pt
		self.change = True
		player = self.NG.pl[self.cp]
		self.NM.pp = player.pos
		self.score = player.score
		self.look()


def my_hook():
	print "shutting down"


rospy.on_shutdown(my_hook)
start = Decisions()