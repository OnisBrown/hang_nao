#!/usr/bin/env python2.7
import sys
import naoqi
from naoqi import ALProxy
import time
import almath
import rospy
import roslib
from move_naoqi import Mover
import game
from hang-nao.msg import GameState, PlayerState

# message on program exit
def my_hook():
	print "shutting down"


rospy.on_shutdown(my_hook)

NM = Mover()
NM.body_reset()
NG = game.HangMan()

def yes(score):
	if score < 0.8:
		NM.head_nod(score)
	else:
		NM.head_nod()
		NM.cheer()
		NM.body_reset()
	return

def no(score):
	NM.head_shake(score)
	return

def victory():
	NM.cheer()
	NM.cheer()

def defeat(score):
	NM.head_shake(score)
	NM.head_shake(score)

def answer(response):
	playerID = response.pt
	score = NG.pl[playerID].score
	if response.turn != 0:
		if bool(response.verify):
			yes(score)
		if not bool(response.verify):
			no(score)
	else:
		if bool(response.win):
			victory()
		else:
			defeat(score)

#def mover(mc):
#	if mc == 0:
#		NM.body_reset()
#		return
#	if mc == 1:
#		NM.head_shake(mood)
#		return
#	if mc == 2:
#		NM.head_nod(mood)
#		return
#	if mc == 3:
#		NM.cheer()
#		NM.cheer()
#		return
#	else:
#		print "Invalid movement"
#		return

rospy.init_node('core', anonymous=True)
rospy.Subscriber('/game/GameState', GameState, answer)
NG.game_start()
#rospy.Subscriber('/game/PlayerState', GameState, update)

#while not rospy.is_shutdown():
#	try:
#		c = raw_input('would you like to move?')
#		if c == 'y':
#			mood = float(raw_input("mood: "))
#			choice = int(raw_input("select movement "))
#			print choice
#			mover(choice)
#		else:
#			NG.game_start()

#	except KeyboardInterrupt:
#		sys.exit()
