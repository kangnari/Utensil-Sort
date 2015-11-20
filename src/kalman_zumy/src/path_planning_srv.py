#!/usr/bin/env python
import rospy
import sys
import tf
from sensor_msgs.msg import Image
from kalman_zumy.srv import ImageSrv, ImageSrvResponse, PathPlanningSrv, PathPlanningSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *

""" Takes a destination AR tag and requests a global image to
plan a path for the Zumy to the destination. Returns the path
as a list of coordinate tuples that the Zumy should sequentially
follow.

In the path planning avoidance algorithm, any blocks near and or
under objects should be penalized.

Additionally, the length of the path and the number of turns 
should be minimized.
"""
class PathPlanningService:
    """ Initializes the node.

    Keyword arguments:
    x_length -- length of x-boundary in cm [float]
    y_length -- length of y-boundary in cm [float]
    num_x    -- number of x points in grid [int]
    num_y    -- number of y points in grid [int]
    """
    def __init__(self, x_length, y_length, num_x, num_y):
        #Initialize the node
        rospy.init_node('marco_polo')
        # Create a CvBridge to convert ROS messages to OpenCV images
        self.bridge = CvBridge()
        # The last image provided by the usb_cam_utensil/raw_image topic
        self.last_image = None
        # Set the boundary corners for Zumy movement in the image plane.
        # Should be a numpy array.
        self.boundary_corners = []
        # Set the x/y boundary lengths
        self.x_length = x_length
        self.y_length = y_length
        # Set the number of x/y points in the movement grid
        self.nx = num_x
        self.ny = num_y
        # Set up listener for tf.Transform
        self.listener = tf.TransformListener()

        #Create the service
        rospy.Service('plan_path', PathPlanningSrv, self.getPath)

    """ Create a grid of points for the Zumy to follow.
    Associate u,v image coordinates with x,y real-world coordinates.
    """
    def create_homography():
        for i in range(nx + 1):
            for j in range(ny + 1):
                # Do some amazing math shit here

    """ Converts a ROS Image message to a NumPy array to be
    displayed by OpenCV. Taken directly from EE106A lab 4.

    Keyword arguments:
    ros_img_msg -- the ROS Image message [sensor_msgs.msg.Image]
    """
    def ros_to_np_img(self, ros_img_msg):
      return np.array(self.bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

    """ Calculate the path the Zumy should take from
    the origin tag to the end goal tag.
    """
    def getPath(self, request):
        # Set up a function to call the image capture service. The
        # service is set up in camera_srv.py and uses the ImageSrv
        # service type.
        last_image_service = rospy.ServiceProxy('last_image_global', ImageSrv)

        # Set last_image to the NumPy converted image
        self.last_image = self.ros_to_np_img(last_image_service().image_data)

        # destination = request.destination_ar
        # origin = request.origin_ar
        return PathPlanningSrvResponse([(10, 20), (40, 20), (10, 10)])

    """ Main node execution function.
    """
    def run(self):
        # Waits for the image service to become available.
        # This service is set up in camera_srv.py.
        rospy.wait_for_service('last_image_global')

        # Create the image capture function and save image
        last_image_service = rospy.ServiceProxy('last_image_global', ImageSrv)
        self.last_image = self.ros_to_np_img(last_image_service().image_data)

        points = []
        corners = ["top left", "top right", "bottom left", "bottom right"]
        # Callback function taken from lab4 code. Adds a clicked
        # point to the points list.
        def on_mouse_click(event, x, y, flag, param):
            if (event == cv2.EVENT_LBUTTONUP):
                point = (x, y)
                points.append(point)
                if len(points) < 4:
                    print "Click on the " + corners[len(points)] + " boundary corner."
                else:
                    print "Done calibrating image."

        # Display the CV Image
        cv2.imshow("CV Image", self.last_image)
        print "Click on the " + corners[0] + " boundary corner."
        cv2.setMouseCallback("CV Image", on_mouse_click, param=1)

        # Have user choose the four boundary corners for
        # Zumy movement 
        while len(points) < 4:
            if rospy.is_shutdown():
                raise KeyboardInterrupt
            cv2.waitKey(10)

        self.boundary_corners = np.array(points)

        # Done with setup. Wait for path planning requests.
        rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("Use: path_planning.py [ x length ] [ y length ] [ # x points in grid ] [ # y points in grid ]")
        sys.exit()
    node = PathPlanningService(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    node.run()
