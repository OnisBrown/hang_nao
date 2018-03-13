#!/usr/bin/env python2.7

import sys
import time

class Player:
	def __init__(self):
		self.score = 0
		self.names = []
		self.faces = []
		

class HangMan:  # code built on example from http://www.pythonforbeginners.com/code-snippets-source-code/game-hangman
	def __init__(self):
		self.name = raw_input("What is your name? ")



	def game(self):

		print "Hello, " + self.name, "Time to play hangman!"
		print ""
		time.sleep(1)
		print "Start guessing..."
		time.sleep(0.5)

		#here we set the secret
		word = "secret"

		#creates a variable with an empty value
		guesses = ''

		#determine the number of turns
		turns = 8

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
				print "You won"
				break

			print "next round"

			# ask the user go guess a character
			guess = raw_input("guess a character:")

			# set the players guess to guesses
			guesses += guess

			# if the guess is not found in the secret word
			if guess not in word:
				turns -= 1

			# print wrong
				print "Wrong"

			# how many turns are left
				print "You have", + turns, 'more guesses'

			# if the turns are equal to zero
				if turns == 0:

					print "You Loose"
