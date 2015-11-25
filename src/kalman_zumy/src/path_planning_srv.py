#!/usr/bin/env python
import rospy
import sys
import tf
from sensor_msgs.msg import Image
from kalman_zumy.srv import ImageSrv, ImageSrvResponse
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
    debug    -- true if check_homography should run
    """
    def __init__(self, x_length, y_length, num_x, num_y, debug):
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
        # Homograhy matrix
        self.H = None
        self.debug = debug

        #Create the service
        #rospy.Service('plan_path', PathPlanningSrv, self.getPath)
        
    # for debugging convert from string to int
    def check_homography(self):
        dx = self.x_length/self.nx
        dy = self.y_length/self.ny
        for i in range(self.nx + 1):
            for j in range(self.ny + 1):
                    xbar = np.array([[i*dx], [j*dy], [1]])
                    ubar = np.dot(self.H, xbar).T[0]
                    u = np.int(ubar[0]/ubar[2])
                    v = np.int(ubar[1]/ubar[2])
                    cv2.circle(self.last_image, (u, v), 5, 0, -1)
            print "dOT LOCATION: " + str((u, v))
        cv2.imshow('Check homography', self.last_image)
    cv2.waitKey(1000)

    """ Create a grid of points for the Zumy to follow.
    Associate x,y real-world coordinates with u,v image coordinates.
    """
    def create_homography(self):
        # Define x,y ground coordinates
        g_coords = np.array([[0, 0], [self.x_length, 0], [self.x_length, self.y_length], [0, self.y_length]])
        A = np.zeros([8, 8])
        b = np.array([])
        i = 0
        uv = self.boundary_corners
        for coord in g_coords:
            u = uv[0, i]
            v = uv[1, i]
            x = coord[0]
            y = coord[1]
            b = np.append(b, [u, v])
            array1 = np.array([x, y, 1, 0, 0, 0, -u*x, -u*y])
            array2 = np.array([0, 0, 0, x, y, 1, -v*x, -v*y])
            A[2*i] = array1
            A[2*i+1] = array2
            i += 1
        x = np.append(np.dot(np.linalg.inv(A), b.T), 1)
        self.H = x.reshape(3, 3)

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
        #return PathPlanningSrvResponse([(10, 20), (40, 20), (10, 10)])

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
        corners = ["bottom left", "bottom right", "top right", "top left"]
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

        cv2.startWindowThread()
        cv2.namedWindow("CV Image")
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

        # Exit out of picture
        cv2.destroyAllWindows()

        # | u1 u2 u3 u4 |
        # | v1 v2 v3 v4 |
        self.boundary_corners = np.array(points).T
        self.create_homography()

        if self.debug:
            self.check_homography()

        # Done with setup. Wait for path planning requests.
        rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("Use: path_planning.py [ x length ] [ y length ] [ # x points in grid ] [ # y points in grid ] [ debug? (optional]")
        sys.exit()
    debug = False
    node = PathPlanningService(float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), debug)
    node.run()
