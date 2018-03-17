#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import httplib, urllib
import time
sleep = 30 # how many seconds to sleep between posts to the channel
key = 'OD4Z4L1JTE4HBR3L'  #Plug&Grow

def pub(data,field):

    fieldname = 'field' + str(field)
    params = urllib.urlencode({fieldname: data, 'key':key }) 
    headers = {"Content-typZZe": "application/x-www-form-urlencoded","Accept": "text/plain"}
    conn = httplib.HTTPConnection("api.thingspeak.com:80")

    conn.request("POST", "/update", params, headers)
    response = conn.getresponse()
    print (response.status, response.reason)
    data = response.read()
    conn.close()
    rospy.loginfo('Publicou canal: ' + fieldname)
    #rospy.loginfo('Tenta Dormir...')  
    if field == 1:
        time.sleep(180)
    elif field == 2:
        time.sleep(120)
    elif field == 3:
        time.sleep(60)

def ambTempPub(data):    
    ambTemp = data.data
    rospy.loginfo('ambTemp = %s', ambTemp)
    time.sleep(30)
    pub(ambTemp,1)

def soilTempPub(data):
    rospy.loginfo('soilTemp = %s', data.data)
    time.sleep(60)
    pub(data.data,2)

def soilMoistPub(data):
    soilMoist = data.data
    rospy.loginfo('soilMoist = %s', soilMoist)
    time.sleep(120)
    pub(soilMoist,3)

def ambLightPub(data):
    rospy.loginfo('AmbLight = %s', data.data)
    time.sleep(180)
    pub(data.data,4)

def thingspeak_pub():
    rospy.init_node('thingspeak_pub', anonymous=True)
    rospy.Subscriber('ambTemperature', Float32, ambTempPub, queue_size = 1)
    rospy.Subscriber('soilTemperature', Float32, soilTempPub, queue_size = 1)
    rospy.Subscriber('soilMoisture', Int32, soilMoistPub, queue_size = 1)
    rospy.Subscriber('ambLight', Int32, ambLightPub, queue_size = 1)
    rospy.spin()

if __name__ == "__main__":
    thingspeak_pub()          

