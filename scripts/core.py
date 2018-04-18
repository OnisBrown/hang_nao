import sys
#import naoqi
import time
#import almath
import rospy
import roslib
from movement import Mover
import game
from hang_nao.msg import GameState, NewTurn
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from control_msgs.msg import JointTrajectoryControllerState
from threading import Timer, Thread, Lock
from random import uniform, randint
from decimal import *
import numpy

class Decisions:
	def __init__(self):
		# 1280x960 resolution
		# 60.9d ~ 1.062906r HFOV | 47.6d ~ 0.8307767r VFOV
		# 0.00083039531r per horizontal pixel | 0.00086539239 per vertical pixel
		self.unitX = Decimal(0.00083039531)
		self.unitY = Decimal(0.00086539239)
		self.NM = Mover()
		self.NG = game.HangMan()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("rgb", 1)
		self.image = []
		self.cp = 0
		self.score = 0.7
		self.bgp = True
		self.change = True
		self.tracking = False
		self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		rospy.Subscriber('/nao_robot/camera/top/image_raw', Image, self.head_view)
		self.NM.body_reset()
		self.idle_lock = Lock()
		pan_start = Thread(target=self.pan)
		pan_start.start()
		print "Panning for players"
		pan_start.join()
		# initialise game subscribers and start the game
		self.playing = True
		idleThread = Thread(target=self.idling)
		idleThread.start()
		rospy.Subscriber('/game/GameState', GameState, self.answer)
		rospy.Subscriber('/game/NewTurn', NewTurn, self.update_turn)
		self.NG.game_start()


	def pan(self):
		angle = -1
		found = 0
		tol = 400
		self.NM.target([0, -1])
		time.sleep(1)
		while angle < 1 and found < len(self.NG.pl):
			try:
				self.NM.target([0, angle])
				faces = self.face_detect()
				for (x, y, w, h) in faces:
					Fpos = self.NG.pl[found].pos
					cv2.rectangle(self.image, (x, y), (x + w, y + h), (255, 0, 0), 2)
					# gets coordinates based on centre of the face found
					x += w / 2
					y += h / 2

					diffX = float(((639-x)*self.unitX))
					diffY = float(((y - 479)*self.unitY))

					Fpos[1] = self.NM.HX + diffX

					Fpos[0] = self.NM.HY + diffY

					# if face is within tolerance of already acquired skips it
					if found == 0:
						self.NG.pl[found].pos = Fpos
						print "new at " + str(Fpos) + "with a tolerance of " + str(tol * self.unitX)
						found += 1
						break

					elif (self.NG.pl[found-1].pos[1] - float(tol * self.unitX)) > Fpos[1] or Fpos[1] > (self.NG.pl[found-1].pos[1] + float(tol * self.unitX)):
						self.NG.pl[found].pos = Fpos
						print "new at " + str(Fpos) + "with a tolerance of " + str(tol * self.unitX)
						found += 1
						break

				angle += float(100*self.unitX) # angle to move at
			except KeyboardInterrupt:
				self.shutdown()

		print "acquired " + str(found) + " players"
		for p in self.NG.pl:
			print str(p.pos)

		raw_input("whelp")

	def shutdown(self):
		self.playing = False
		sys.exit()

	def head_view(self, img):
		self.image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
		# goes through all faces in view checking the location of the current players face face
		tol = 200 # set tolerence for
		faces = self.face_detect()
		Fpos = self.NG.pl[self.cp].pos
		temp = [0.0, 0.0]
		diffX = list()
		diffY = list()
		if self.tracking:
			self.look(Fpos)
			for (x, y, w, h) in faces:
				cv2.rectangle(self.image, (x, y), (x + w, y + h), (255, 0, 0), 2)
				# gets coordinates based on centre of the face found
				x += w / 2
				y += h / 2

				temp[0] = self.NM.HY + float(((y - 479) * self.unitY))
				temp[1] = self.NM.HX + float(((639 - x) * self.unitX))

				if (Fpos[0] - float(tol*self.unitY)) < temp[0] < (Fpos[0] + float(tol*self.unitY)):
					if (Fpos[1] - float(tol * self.unitY)) < temp[1] < (Fpos[1] + float(tol * self.unitY)):
						diffX.append(float(((639 - x) * self.unitX)))
						diffY.append(float(((y - 479) * self.unitY)))

		# finds the closest face to the original position
			if len(diffX) > 0 and len(diffY) > 0:
				Fpos[1] = self.NM.HX + min(diffX, key=abs)
				Fpos[0] = self.NM.HY + min(diffY, key=abs)

			self.NG.pl[self.cp].pos = Fpos
			self.NM.pp = self.NG.pl[self.cp].pos

		cv2.imshow("rgb", self.image)
		cv2.waitKey(3)

	def face_detect(self):
		gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		faces = self.face_cascade.detectMultiScale(gray, 1.1, 10)
		return faces

	def idling(self):
		if self.playing:
			try:
				if self.change:
					self.trace()
					self.look_away()

				self.idling()

			except KeyboardInterrupt:
				self.shutdown()

	def trace(self):
		self.tracking = True
		lt = uniform(2.5, 3.5) + self.score  # maintains gaze for random time based on mutual gaze data
		time.sleep(lt)

	def look(self, pos):
		self.NM.target(pos)

	def look_away(self):
		temp = [0]
		if self.bgp:
			self.bgp = False

			for i in self.NG.pl:  # goes through player list picking out the best guesser(s)
				if i.cg > 0:
					if i.cg == temp[0]:
						temp.append(i.pos)

					elif i.cg > temp[0]:
						temp = temp[:1]
						temp[0] = i.pos

			if temp[0] != 0: # if the temp list is unchanged then the
				if len(temp) == 1:
					bestp = temp[0]
				else:
					bestp = temp[randint(0, 1)]

				if bestp != self.NM.pp: # if the bot decides that the current player is the best then code moves on
					self.look(bestp)
					bt = uniform(2.0, 2.5) - self.score  # time before nao looks somewhere else
					time.sleep(bt+0.5)
					return

		self.NM.idle()
		bt = uniform(2.0, 2.5) - self.score  # time before nao looks somewhere else
		time.sleep(bt)

	def update_turn(self, newturn):
		print "next turn"

		self.bgp = True
		self.cp = newturn.pt
		self.change = True
		self.idle = True
		player = self.NG.pl[self.cp]
		self.NM.pp = player.pos
		print str(player.pos)
		self.score = player.score

	# answer function
	def answer(self, response):
		self.change = False
		self.NM.target()
		if response.turn > 0:
			if bool(response.win):
				self.playing = False
				self.victory()

			else:
				if response.verify == 1:
					self.yes()
					return

				elif response.verify == 0:
					self.no()
					return

		else:
			self.playing = False
			self.defeat()

		self.change = True

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
		self.look([0, 0])

	def defeat(self):
		self.NM.head_shake(0)
		self.NM.head_shake(0)
		self.look([0, 0])

def my_hook():
	print "shutting down"

try:
	rospy.on_shutdown(my_hook)
	start = Decisions()
except KeyboardInterrupt:
	sys.exit()