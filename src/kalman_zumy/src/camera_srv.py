#!/usr/bin/env python
import rospy
import sys

from sensor_msgs.msg import Image
from kalman_zumy.srv import ImageSrv, ImageSrvResponse

""" Provides the last snapshot taken from the appropriate
usb_cam to whatever node requests it. The image_process node
should be the only node requesting images from usb_cam_utensil/image_raw.
The main_control and path_planning nodes should be the only nodes
requesting images from usb_cam_global.

Taken directly from EE106A lab 4.
"""
class ImgService:
    def __init__(self, cam_name):
        #Create an instance variable to store the last image received
        self.lastImage = None;

        #Initialize the node
        rospy.init_node('cam_listener')

        #Subscribe to the image topic
        rospy.Subscriber("/usb_cam_" + cam_name + "/image_raw", Image, self.imgReceived)

        #Create the service
        rospy.Service("last_image_" + cam_name, ImageSrv, self.getLastImage)

    #Callback for when an image is received
    def imgReceived(self, message):
        #Save the image in the instance variable
        self.lastImage = message

    #When another node calls the service, return the last image
    def getLastImage(self, request):
        #Return the last image
        return ImageSrvResponse(self.lastImage)

    def run(self):
        rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Use: camera_srv.py [ utensil/global ]")
        sys.exit()
    node = ImgService(sys.argv[2])
    node.run()
