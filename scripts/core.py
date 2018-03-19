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
from hang_nao.msg import GameState, PlayerState, NewTurn

# message on program exit
def my_hook():
	print "shutting down"


rospy.on_shutdown(my_hook)

NM = Mover()
NG = game.HangMan()

def yes(score):
	if score < 0.8:
		NM.head_nod()
	else:
		NM.head_nod()
		NM.cheer()
		NM.body_reset()
	return

def no(score):
	NM.head_shake()
	return

def victory():
	NM.cheer()
	NM.cheer()

def defeat(score):
	NM.head_shake()
	NM.head_shake()

def look():
	NM.target()

def answer(response):
	playerID = NG.cp
	NM.score = NG.pl[playerID].score
	NM.pp = NG.pl[playerID].pos
	if response.turn > 0:
		if bool(response.win):
			victory()
		else:
			if response.verify == 1:
				yes()
				#look(pp)
			if response.verify == 0:
				no()
				#look(pp)
			if response.verify == 2:
				look()
	else:
		defeat()

def update_turn(newturn):
	NM.change = True

rospy.Subscriber('/game/GameState', GameState, answer)
NG.game_start()
rospy.Subscriber('/game/NewTurn', NewTurn, update_turn)

