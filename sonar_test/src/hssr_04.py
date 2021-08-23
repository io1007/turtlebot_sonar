#! /usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range 
import RPi.GPIO as GPIO
import time
import sys

class Sonar():

    def __init__(self,radiation_type=0,min_range=0.02,max_range=4.00):
        
        #Define variables
        self.radiation_type = radiation_type
        self.min_range = min_range
        self.max_range = max_range
        self.h = Header()

        #Initialize node
        rospy.init_node("sonar_sensor")
        #Initialize Publisher
        self.distance_pub = rospy.Publisher("sonar_dist",Range,queue_size=1)
        #loop_rate
        self.rate = rospy.Rate(1)

    def dist_sendor(self,dist):
        msg = Range()
        msg.range = dist
        msg.radiation_type = self.radiation_type
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        self.h.stamp = rospy.Time.now()
        msg.header = self.h
        self.distance_pub.publish(msg)
        rospy.loginfo( "Distance:{}cm".format(str(msg.range)))

    def get_sonar(self):

        GPIO.setmode(GPIO.BCM)

        TRIG = 17 
        ECHO = 27

        print "Distance Measurement In Progress"

        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)


        while not rospy.is_shutdown():

            GPIO.output(TRIG, False)
            print "Waiting For Sensor To Settle"
            time.sleep(2)

            GPIO.output(TRIG, True)
            time.sleep(0.00001)
            GPIO.output(TRIG, False)

            while GPIO.input(ECHO)==0:
              pulse_start = time.time()

            while GPIO.input(ECHO)==1:
              pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)

            self.dist_sendor(distance)


            self.rate.sleep()

            #GPIO.cleanup(TRIG)
            #GPIO.cleanup(ECHO)

if __name__ == '__main__':
    try:
        sensor = Sonar()
        sensor.get_sonar()
    except rospy.ROSInterruptException:
        GPIO.cleanup(TRIG)
        GPIO.cleanup(ECHO)
        sys.exit(0)
