#!/usr/bin/env python
"""
Simple example of subscribing to sensor messages and publishing
twist messages to the turtlebot.

Author: Andrew Rozniakowski
Version: 1/22/2015

"""
import rospy

# Twist is the message type for sending movement commands.
from geometry_msgs.msg import Twist


class announce(object):

    # globals
    twist = None
    
    # This function will be called every time a new twist message is
    # published.
    def twist_callback():

        # Creates new Global Twist object
        global twist
        twist = Twist()

    def __init_(self):
       
        # Turn this into an official ROS node named announce
        rospy.init_node('announce')
        
        #Suscribing to the twist message 
        rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, twist_callback)
        
        #Publishing to robotsound    
        pub = rospy.Publisher('/robotsound', sound_play) 

        #Creating sound play object
        sound = sound_play()

        while not rospy.is_shutdown():

            #Publishing sound request if robot is reversing  
            if twist.linear.x < 0:
                print "This is kinda Working"
                pub.publish(sound)      

