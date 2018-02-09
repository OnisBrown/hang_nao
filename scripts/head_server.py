#!/usr/bin/env python

from hang_nao.srv import *
import rospy

def head_server():
	rospy.init_node('head_server')
	s = rospy.Service('add_two_ints', head)
	print "moving head"
	rospy.spin()

if __name__ == "__main__":
	head_server()