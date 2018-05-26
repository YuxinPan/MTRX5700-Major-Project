#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os.path

def talker():
	pub = rospy.Publisher('/camera/image_raw', Image,
				queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(15) # 2hz
	#capture = cv2.VideoCapture(0)

	br = CvBridge()
	counter = 0
	while not rospy.is_shutdown():
		#[status,img] = capture.retrieve()
		#if status == True:

		filename = '/home/yuxin/share/rgb/frame'+str(counter+2)+'.jpg'
		while not os.path.isfile(filename):
			rate.sleep()
		counter +=1
		filename = '/home/yuxin/share/rgb/frame'+str(counter)+'.jpg'
		img = cv2.imread(filename)
		rospy.loginfo('publish image')
		pub.publish(br.cv2_to_imgmsg(
					img))
		try:
    			os.remove(filename)
		except OSError:
    			pass

		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		exit
