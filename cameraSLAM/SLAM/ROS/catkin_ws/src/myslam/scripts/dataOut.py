#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
#import tf.tfMessage as dataType
#import turtlesim.srv

def dataHandler(data):
	print(data)



if __name__ == '__main__':

	counter = 0

	rospy.init_node('listener', anonymous=True)

  	listener = tf.TransformListener()


	##rospy.Subscriber('/tf', tf, dataHandler)             # it finds the newest message


	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/camera','/world', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		#angular = 4 * math.atan2(trans[1], trans[0])
		#linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
		#cmd = geometry_msgs.msg.Twist()
		#cmd.linear.x = linear
 		#cmd.angular.z = angular
  		#turtle_vel.publish(cmd)
		#print('angular')
		#print(angular)
		#print('linear')
		#print(linear)
		print("translation")
		print(trans)
		print("rotation")
		print(rot)
		
		counter += 1
		
		F = open("/home/yuxin/share/textdata/data"+str(counter)+".txt","w")

		F.write(str(trans))
		F.write("\n")
		F.write(str(rot))
		F.close


		rate.sleep()
