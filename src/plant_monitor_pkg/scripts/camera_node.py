#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Int32
import picamera
import datetime
import time

from time import sleep

def callback(data):
    global ambLight
    ambLight = data.data
    #print 'luminosidade: ' + str(ambLight)

def camera_node():
    rospy.init_node('camera_node', anonymous=True)
    rospy.Subscriber('ambLight', Int32, callback)
    minute_rate = 45
    rate = rospy.Rate(1.0/(minute_rate*60.0)) #Hz

    rospy.loginfo("Iniciando Camera_node...")
    sleep(60)

    while not rospy.is_shutdown():
        date = datetime.datetime.now()
        filename = "/home/tiago/Pictures/Plant/" + str(date) + ".jpg"
        
        if ambLight<10:        
        #if date.hour > 19 or date.hour < 6:
            rospy.loginfo("Too dark, no picture.")
            #camera = picamera.PiCamera()
			#camera.resolution = (2592, 1944)
			#camera.resolution = (1280, 720)
            #camera.resolution = (640, 480)
            #camera.exposure_mode = 'verylong'
            #camera.framerate = 1
            #camera.shutter_speed = 7000000
            #camera.iso = 1600
            #camera.capture(filename)
            #rospy.loginfo("Desliga Lampada...")
            #camera.close()
        else:
            camera = picamera.PiCamera()
            camera.resolution = (640, 480)
            camera.capture(filename)
            camera.close()
            rospy.loginfo("Foto!")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
