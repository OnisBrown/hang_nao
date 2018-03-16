#!/usr/bin/env python2.7
import sys
import naoqi
from naoqi import ALProxy
import time
import almath
import rospy
import roslib
from move_naoqi import Mover
from game import *


# message on program exit
def my_hook():
	print "shutting down"


rospy.on_shutdown(my_hook)

NM = Mover()
NG = HangMan()
mood = 0.3


def mover(mc):
	if mc == 0:
		NM.body_reset()
		return
	if mc == 1:
		NM.head_shake(mood)
		return
	if mc == 2:
		NM.head_nod(mood)
		return
	if mc == 3:
		NM.cheer()
		NM.cheer()
		return
	else:
		print "Invalid movement"
		return


while not rospy.is_shutdown():
	try:

		mood = float(raw_input("mood: "))
		choice = int(raw_input("select movement "))
		print choice
		mover(choice)

		#NG.game()

	except KeyboardInterrupt:
		sys.exit()
