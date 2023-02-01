#!/usr/bin/env python
"""
Node for handling back-up warnings.

Subscribes to:
 '/cmd_vel_mux/input/navi'

Publishes to:
  '/robotsound'

Author: Nathan Sprague
Version: 1/15

"""
import rospy
import math

from geometry_msgs.msg import Twist
from sound_play.msg import SoundRequest


class AnnounceNode(object):

    def __init__(self):
        """ Set up the node. """
        rospy.init_node('announce')
        rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, self.cmd_callback)
        self.sound_pub = rospy.Publisher('/robotsound', SoundRequest)
        self.twist = None

    def cmd_callback(self, twist):
        """ Store twist messages. """
        self.twist = twist

    def main_loop(self):
        """ Monitor velocity, and announce warnings if needed. """
        sound_request = SoundRequest()
        sound_request.sound = SoundRequest.SAY
        sound_request.command = SoundRequest.PLAY_ONCE
        sound_request.arg = "backing up"

        while not rospy.is_shutdown():
            if self.twist is not None and self.twist.linear.x < 0:
                self.sound_pub.publish(sound_request)
                rospy.sleep(1.0)
            else:
                rospy.sleep(.1)

if __name__ == "__main__":
    announce = AnnounceNode()
    announce.main_loop()
