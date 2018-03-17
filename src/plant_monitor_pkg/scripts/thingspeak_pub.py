#!/usr/bin/env python

import rospy
from plant_monitor_pkg.msg import sensors

import httplib, urllib
import time
sleep = 5*60 # how many seconds to sleep between posts to the channel
key = 'OD4Z4L1JTE4HBR3L'  #Plug&Grow

def pub(data):

    params = urllib.urlencode({'field1': data.ambtemp,'field2': data.soiltemp,'field3': data.soilmoist,'field4': data.light,'field5': data.ambHumidity, 'key':key }) 
    headers = {"Content-typZZe": "application/x-www-form-urlencoded","Accept": "text/plain"}
    conn = httplib.HTTPConnection("api.thingspeak.com:80")

    conn.request("POST", "/update", params, headers)
    response = conn.getresponse()
    print (response.status, response.reason)
    data = response.read()
    conn.close()
    rospy.loginfo('Publicou no ThingSpeak! ')
    time.sleep(sleep)
    
def thingspeak_pub():
    time.sleep(60)
    rospy.loginfo('Iniciou ThingSpeak publisher...')
    rospy.init_node('thingspeak_pub', anonymous=True)
    
    rospy.Subscriber('filtered_signals', sensors, pub, queue_size = 1)    
    rospy.spin()

if __name__ == "__main__":
    thingspeak_pub()          

