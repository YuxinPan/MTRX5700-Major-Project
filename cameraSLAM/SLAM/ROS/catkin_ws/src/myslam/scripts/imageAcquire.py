#!/usr/bin/env python
import rospy,signal,sys
from std_msgs.msg import String

import serial,time
import decimal
# port config

def signal_handler(signal, frame):
        print('\nCtrl+C detected, exiting program')
        sys.exit(0)


def talker():
	print('Sending images')
	pub = rospy.Publisher('imageFeed', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	#while not rospy.is_shutdown():
	#	hello_str = s+";Time:%s" % rospy.get_time()
	#	rospy.loginfo(hello_str)
	#	pub.publish(hello_str)
	rate.sleep()


signal.signal(signal.SIGINT, signal_handler)


while True:
	talker()
