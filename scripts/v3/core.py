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
import numpy

class Decisions:
	def __init__(self):
		# 1280x960 resolution
		# 60.9d ~ 1.062906r HFOV | 47.6d ~ 0.8307767r VFOV
		# 0.00083039531r per horizontal pixel | 0.00086539239 per vertical pixel
		self.unitX = 0.00083039531
		self.unitY = 0.00086539239
		self.tol = 300
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
		rospy.Subscriber('/nao_dcm/Head_controller/state', JointTrajectoryControllerState, self.head_update)
		self.HY = 0.0
		self.HX = 0.0
		self.NM.body_reset()
		self.idle_lock = Lock()
		pan_start = Thread(target=self.pan)
		pan_start.start()
		print "Panning for players"
		pan_start.join()
		# initialise game subscribers and start the game
		rospy.Subscriber('/game/GameState', GameState, self.answer)
		rospy.Subscriber('/game/NewTurn', NewTurn, self.update_turn)
		self.NG.game_start()

	def pan(self):
		angle = -1
		found = 0
		self.NM.target([0, -1])
		print str(len(self.NG.pl))
		while angle < 1 and found < len(self.NG.pl):
			try:
				print str(found)
				self.NM.target([0, angle])
				faces = self.face_detect()
				for (x, y, w, h) in faces:
					new = True
					Fpos = self.NG.pl[found].pos
					# gets coordinates based on centre of the face found
					x += w / 2
					y += h / 2

					if x > 639:
						Fpos[1] = self.HX - ((x-639)*self.unitX)
					else:
						Fpos[1] = self.HX + ((639-x)*self.unitX)

					if y > 479:
						Fpos[0] = 0 + ((y-479)*self.unitY)
					else:
						Fpos[0] = 0 - ((479-y)*self.unitY)

					# if face is within 5 degrees of already acquired skips it

					#for i in self.NG.pl:
					if found > 0:
						prevF = self.NG.pl[found-1].pos
						if (prevF[0] + self.tol*self.unitY) < Fpos[0] < (prevF[0] - self.tol*self.unitY):
							if (prevF[1] + self.tol*self.unitX) < Fpos[1] < (prevF[1] - self.tol*self.unitX):
								continue

					self.NG.pl[found].pos = Fpos
					#print "new at " + str(Fpos) + "around " + str(prevF) + "with a tolerance of " + str(self.tol*self.unitX)
					found += 1

				angle += 100*self.unitX # moves in increments of 4 degrees
			except KeyboardInterrupt:
				sys.exit()

		print "acquired " + str(found) + " players"
		for p in self.NG.pl:
			print str(p.pos)

		raw_input("whelp")

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
		try:
			self.idle_lock.release()
		except Exception, e:
			print ""
		self.change = False
		self.tracking = False
		if response.turn > 0:
			if bool(response.win):
				self.victory()

			else:
				if response.verify == 1:
					self.yes()
					return

				if response.verify == 0:
					self.no()
					return

		else:
			self.defeat()

		self.change = True

	def face_detect(self):
		gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		faces = self.face_cascade.detectMultiScale(gray, 1.4, 4)
		return faces

	def head_view(self, img):
		self.image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
		# goes through all faces in view checking the location of the current players face face
		if self.tracking:
			faces = self.face_detect()
			Fpos = self.NG.pl[self.cp].pos
			diff = 900000
			for (x, y, w, h) in faces:
				cv2.rectangle(self.image, (x, y), (x + w, y + h), (255, 0, 0), 2)
				# gets coordinates based on centre of the face found
				x += w / 2
				y += h / 2

				# if face is within 5 degrees of old location lets

				if (Fpos[1] - self.tol*self.unitX) < x < (Fpos[1] + self.tol*self.unitX):
					if x > 639:
						Fpos[1] = self.HX - ((x-639)*self.unitX)
					else:
						Fpos[1] = self.HX + ((639-x)*self.unitX)

				if (Fpos[0] - self.tol*self.unitY) < y < (Fpos[0] + self.tol*self.unitY):
					if y > 479:
						Fpos[0] = self.HY + ((y-479)*self.unitY)
					else:
						Fpos[0] = self.HY - ((479-y)*self.unitY)

			self.NG.pl[self.cp].pos = Fpos
			self.NM.pp = self.NG.pl[self.cp].pos
			self.NM.target()

		cv2.imshow("rgb", self.image)
		cv2.waitKey(3)

	def trace(self):
		self.idle_lock.acquire()
		self.tracking = True
		lt = uniform(2.5, 3.5) + self.score  # maintains gaze for random time based on mutual gaze data
		ltt = Timer(lt, self.look_away)
		ltt.start()


	def look(self, pos):
		self.NM.target(pos)

	def look_away(self):
		self.idle_lock.release()
		self.tracking = False
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
						self.look(bestp)
						bt = uniform(2.0, 2.5) - self.score  # time before nao looks somewhere else
						btt = Timer(bt, self.trace)
						btt.start()
						time.sleep(bt+0.5)
						btt.cancel()
						return

			self.NM.idle()
			bt = uniform(2.0, 2.5) - self.score  # time before nao looks somewhere else
			btt = Timer(bt, self.trace)
			btt.start()
			time.sleep(bt+0.5)
			btt.cancel()
			return

	def update_turn(self, newturn):
		print "new turn"
		self.NM.body_reset()
		self.bgp = True
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
