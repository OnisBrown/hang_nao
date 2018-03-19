#!/usr/bin/env python2.7

import sys
import time
from random import randint
from hang-nao.msg import GameState, PlayerState
import rospy

class Player:
	def __init__(self):
		self.score = 0.7
		self.id = []
		self.faces = []
		self.pos = []
		

class HangMan:  # code built on example from http://www.pythonforbeginners.com/code-snippets-source-code/game-hangman
	def __init__(self):
		lm = open("medium.txt", "r")
		lh = open("hard.txt", "r")
		am = lm.read()
		ah = lh.read()
		self.sm = am.split()
		self.sh = ah.split()
		self.pl = list()

		#message publishers and message objects
		rospy.init_node('/game', anonymous=True)
		self.gp = rospy.Publisher('/game', GameState, queue_size=100)
		#pp = rospy.Publisher('/game', PlayerState, queue_size=100)
		self.gm = GameState()

	def game_start(self):
		pCount = int(raw_input("How many players are there? "))

		for i in range(pCount):
			self.pl.append(Player())
			self.pl[i].id = i

		hard = raw_input("play on hard mode? (y/n) ")

		wm = self.sm[randint(0, 212)]
		wh = self.sh[randint(0, 212)]

		if hard == "y":
			word = wh
		else:
			word = wm

		print "Time to play hangman!"
		print ""
		time.sleep(0.5)
		print "Start guessing..."
		time.sleep(0.5)

		#creates a variable with an empty value
		guesses = ''
		misses = ''
		#determine the number of turns
		turns = 12

		cp = 0

		try:
			#check if the turns are more than zero
			while turns > 0:
				# make a counter that starts with zero
				failed = 0
				# for every character in
				for char in word:
					if char in guesses:
						print char,

					else:
						print "_",
						failed += 1

				if failed == 0:
					print "\nYou won"
					self.gm.win = 1
					break

				print "\n next round"

				# ask the user go guess a character
				self.gm.pt = self.pl[cp].id
				self.gp.publish(self.gm)
				print "Player " + self.pl[cp].id
				guess = raw_input("guess a character: ")

				if guess == "!": # if the user inputs an exclamation mark exit the game
					break
				# set the players guess to guesses
				guesses += guess

				# if the guess is not found in the secret word
				if guess not in word:
					turns -= len(guess)
					self.pl[cp].score -= 0.1*len(guess)
					self.gm.verify = 0
					if guess not in misses:
						misses += guess

				# print wrong
					print "\n Wrong"

				# how many turns are left
					print "You have", + turns, 'more guesses'

				else:
					self.gm.verify = 1
					self.pl[cp].score += 0.1*len(guess)



				# if the turns are equal to zero
				if turns == 0:
					print "You Loose"
					self.gm.win = 0

				self.gm.turn = turns
				self.gp.publish(self.gm)

				if cp == pCount - 1:
					cp = 0
				else:
					cp += cp

		except KeyboardInterrupt:
			sys.exit()
