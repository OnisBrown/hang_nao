#!/usr/bin/env python2.7

import sys
import time
from random import randint

class Player:
	def __init__(self):
		self.score = 0
		self.names = []
		self.faces = []
		

class HangMan:  # code built on example from http://www.pythonforbeginners.com/code-snippets-source-code/game-hangman
	def __init__(self):
		lm = open("medium.txt", "r")
		lh = open("hard.txt", "r")
		am = lm.read()
		ah = lh.read()
		self.sm = am.split()
		self.sh = ah.split()

	def game(self):
		name = raw_input("What is your name? ")
		hard = raw_input("play on hard mode? (y/n) ")

		wm = self.sm[randint(0, 212)]
		wh = self.sh[randint(0, 212)]

		if hard == "y":
			word = wh
		else:
			word = wm

		print "Hello, " + name, "Time to play hangman!"
		print ""
		time.sleep(1)
		print "Start guessing..."
		time.sleep(0.5)

		#creates a variable with an empty value
		guesses = ''

		#determine the number of turns
		turns = 12

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
					break

				print "\n next round"

				# ask the user go guess a character
				guess = raw_input("guess a character: ")

				if guess == "!": # if the user inputs an exclamation mark exit the game
					break
				# set the players guess to guesses
				guesses += guess

				# if the guess is not found in the secret word
				if guess not in word:
					turns -= 1

				# print wrong
					print "\n Wrong"

				# how many turns are left
					print "You have", + turns, 'more guesses'

				# if the turns are equal to zero
					if turns == 0:
						print "You Loose"

		except KeyboardInterrupt:
			sys.exit()
