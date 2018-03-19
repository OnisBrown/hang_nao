#!/usr/bin/env python2.7

import sys
import time
from random import randint
from hang_nao.msg import GameState, PlayerState
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
		self.gp = rospy.Publisher('/game/GameState', GameState, queue_size=100)
		#pp = rospy.Publisher('/game', PlayerState, queue_size=100)
		self.gm = GameState()

	def game_start(self):
		pCount = int(raw_input("How many players are there? "))

		for i in range(pCount):
			self.pl.append(Player())
			self.pl[i].id = i + 1

		hard = raw_input("play on hard mode? (y/n) ")

		wm = self.sm[randint(0, 212)]
		wh = self.sh[randint(0, 212)]

		if hard == "y":
			word = wh
		else:
			word = wm

		print "Time to play hangman!"
		print ""
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
				print "\n" * 100
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
					self.gm.verify = 2
					self.gp.publish(self.gm)
					raw_input('press enter...')
					break

				print "\n__________________________________________\n"

				# ask the user go guess a character
				self.gm.verify = 2
				self.gm.pt = cp
				self.gp.publish(self.gm)
				print "Player " + str(self.pl[cp].id) + ' your turn'
				print " You have ", + turns, ' guesses remaining'
				print "\nIncorrect guesses: " + misses
				guess = raw_input("\nmake a guess (multiple characters or the word):\n ")
				print "\n"
				if guess == "!": # if the user inputs an exclamation mark exit the game
					break
				# set the players guess to guesses
				guesses += guess

				# if the guess is not found in the secret word
				for char in guess:
					if char not in word:
						turns -= 1
						self.pl[cp].score -= 0.1
						self.gm.verify = 0
						if char not in misses:
								misses += ' ' + char

					# print wrong
						print char + " is wrong"
					else:
						self.gm.verify = 1
						self.pl[cp].score += 0.1

				# how many turns are left
				print "\nYou have", + turns, 'more guesses'





				# if the turns are equal to zero
				if turns == 0:
					print "\nYou Loose"
					self.gm.win = 0

				self.gm.turn = turns
				self.gp.publish(self.gm)

				if cp == pCount - 1:
					cp = 0
				else:
					cp += cp

				for char in word:
					if char in guesses:
						print char,

					else:
						print "_",

				raw_input('\npress enter...')

		except KeyboardInterrupt:
			sys.exit()
