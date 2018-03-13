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
mood = 0.5


def mover(choice):
	switcher = \
	{
		0: NM.body_reset(),
		1: NM.head_shake(),
		2: NM.head_nod()
	}
	switcher.get(choice, "Invalid movement")


while not rospy.is_shutdown():
	try:
		choice = int(raw_input("select movement "))
		mover(choice)

	except KeyboardInterrupt:
		sys.exit()