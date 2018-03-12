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

while not rospy.is_shutdown():
	try:
		#NM.body_reset()
		NM.head_shake(mood)
		#NM.head_nod()

	except KeyboardInterrupt:
		sys.exit()