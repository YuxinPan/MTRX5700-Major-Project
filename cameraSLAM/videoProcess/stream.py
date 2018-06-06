#Create a folder with image files from the video.
#Based on jasper jia code on https://stackoverflow.com/questions/31432870/ros-convert-video-file-to-bag
#Usage: Videotofile.py videopath imagefolder
#imports:
import time, sys, os
#from cv_bridge import CvBridge
import cv2,decimal,math


def userExit():
    # this function does not require global variables to be referenced
    
    if (cv2.waitKey(1) & 0xFF == ord('q')) or (cv2.waitKey(1) & 0xFF == ord('Q')):
        return True
    return False


webcamFrameRate = 30
targetFrameRate = 10
skipFrame = math.floor(decimal.Decimal(30)/10)


counter = 0
cap = cv2.VideoCapture(0)

while True:

        counter +=1

        ret, img = cap.read()
        if counter % skipFrame ==0:
                cv2.imwrite('D:/VirtualBox/share/rgb/frame'+str(int(math.floor(counter/3)))+'.jpg',img)
                cv2.imshow('img',img)
        if userExit():
                break




cap.release()
cv2.destroyAllWindows()

##
##        
##if len(sys.argv) == 3: #if the user has provided enough arguments
##    #extract the arguments
##    videopath = sys.argv[1]
##    imagefolder = sys.argv[2]
##    #run the CreateVideoBag function
##    CreateVideoimages(videopath, imagefolder, False)
##    #voila
##    print "Done"
##    
##elif len(sys.argv) == 4 and sys.argv[3] == "orb-slam2": # It has to be set up for ORB-SLAM2
##    #extract the arguments
##    videopath = sys.argv[1]
##    imagefolder = sys.argv[2] + "/rgb"
##    #run the CreateVideoBag function
##    CreateVideoimages(videopath, imagefolder, True)
##    #voila
##    print "Done"
##else:
##    #The user has not priveded the right amount of arguments, point to this
##    print "Please supply two arguments as following: Videotofiles.py videopath imagefolder [orb-slam2] \n When adding \"orb-slam2\", the nessary text file will also be created."
