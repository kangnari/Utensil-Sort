#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image
from kalman_zumy.srv import ImageSrv, ImageSrvResponse, UtensilSrv, UtensilSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *

""" This node is responsible for requesting an image from 
usb_cam_utensil/raw_image and detecting whether the object
displayed is a fork, knife, or spoon. The main control node
will call this node only when the Zumy is back at the
designated start.

Chopstick detection may become available at some point.
"""
class ImageProcess:
    """ Initializes the node.
    """
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_process')
        # Create a CvBridge to convert ROS messages to OpenCV images
        self.bridge = CvBridge()
        # Set true if the main control node has requested a detection
        self.requestFlag = False
        # The last image provided by the usb_cam_utensil/raw_image topic
        self.last_image = None

        # Create the service for the control client
        rospy.Service('utensil_type', UtensilSrv, self.triggerDetect)

    """ Converts a ROS Image message to a NumPy array to be
    displayed by OpenCV. Taken directly from EE106A lab 4.

    Keyword arguments:
    ros_img_msg -- the ROS Image message [sensor_msgs.msg.Image]
    """
    def ros_to_np_img(self, ros_img_msg):
      return np.array(self.bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

    """ Triggers the detection process. This should only be called when
    an external node makes a request to the utensil_type service.

    Requests an image from the last_image service and detects whether
    the utensil in the image is a knife, fork, or spoon. The result
    is sent back to the requesting node.

    Keyword arguments:
    request -- the request made by another node
    """
    def triggerDetect(self, request):
        # Set up a function to call the image capture service. The
        # service is set up in camera_srv.py and uses the ImageSrv
        # service type.
        last_image_service = rospy.ServiceProxy('last_image', ImageSrv)

        # Set last_image to the NumPy converted image
        self.last_image = self.ros_to_np_img(last_image_service().image_data)

        # Detect what type of utensil is being displayed
        if self.isFork():
            self.utensil_type = "fork"
        elif self.isKnife():
            self.utensil_type = "knife"
        elif self.isSpoon():
            self.utensil_type = "spoon"
        else:
            self.utensil_type = None

        # Notify the requesting node of the utensil type
        return UtensilSrvResponse(self.utensil_type)

    """ Detects whether the utensil in the last image is a fork.
    """
    def isFork(self):
        image = self.last_image
        return

    """ Detects whether the utensil in the last image is a spoon.
    """
    def isKnife(self):
        image = self.last_image
        return

    """ Detects whether the utensil in the last image is a knife.
    """
    def isSpoon(self):
        image = self.last_image
        return

    """ Main node execution function.
    """
    def run(self):
        # Waits for the image service to become available.
        # This servie is set up in camera_srv.py.
        rospy.wait_for_service('last_image')

        # Done with setup. Wait for utensil detection requests.
        rospy.spin()

if __name__ == '__main__':
    node = ImageProcess()
    node.run()
