#!/usr/bin/env python
import rospy
import sys
import numpy as np
from kalman_zumy.srv import *

""" The main control node. Detects when the image_process node
needs to be triggered, and accordingly tells the zumy what needs
to be done.
"""
class MainControl:
    """ Initializes the node.

    Keyword arguments:
    mname -- name of the Zumy (zumyXY) [string]
    """
    def __init__(self, mname, ar_tags):
        # Initialize the node
        rospy.init_node('main_control')
        # Set consistent time intervals for the entire application
        self.hertz = 10
        self.dt = 1/self.hertz
        # Set the rate for sleeping during while loops
        self.rate = rospy.Rate(self.hertz)
        # Set the zumy name
        self.mname = mname
        # Set the provided ar tags
        self.ar_tags = ar_tags

    """ Main node execution function.
    """
    def run(self):
        # Set up a function to call the utensil detection service. The
        # service is set up in image_process.py and uses the UtensilSrv
        # service type.
        rospy.wait_for_service('utensil_type')
        utensil_type = rospy.ServiceProxy('utensil_type', UtensilSrv)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 6:
        print('Use: main_control.py [ zumy name ] [ AR start tag #] [ AR fork tag # ] [ AR knife tag # ] [ AR spoon tag # ]')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['start'] = 'ar_marker_' + sys.argv[2]
    ar_tags['fork']  = 'ar_marker_' + sys.argv[3]
    ar_tags['knife'] = 'ar_marker_' + sys.argv[4]
    ar_tags['spoon'] = 'ar_marker_' + sys.argv[5]
    node = MainControl(zumy_name, ar_tags)
    node.run()
