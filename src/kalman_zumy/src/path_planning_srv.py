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
    """
    def __init__(self):
        #Initialize the node
        rospy.init_node('marco_polo')
        # Create a CvBridge to convert ROS messages to OpenCV images
        self.bridge = CvBridge()
        # The last image provided by the usb_cam_utensil/raw_image topic
        self.last_image = None
        # Set up listener for tf.Transform
        self.listener = tf.TransformListener()

        #Create the service
        rospy.Service('plan_path', PathPlanningSrv, self.getPath)

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
        # This servie is set up in camera_srv.py.
        rospy.wait_for_service('last_image_global')

        # Done with setup. Wait for path planning requests.
        rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    node = PathPlanningService()
    node.run()





# #!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Image
# from kalman_zumy.srv import ImageSrv, ImageSrvResponse
# import cv2, time, sys
# from cv_bridge import CvBridge, CvBridgeError
# import numpy as np
# from numpy.linalg import *


# # Nominal length of a tile side
# # XXX MAY NEED TO CHANGE
# TILE_LENGTH = 30.48 #cm

# # Define the total number of clicks we are expecting (4 corners)
# # XXX MAY NEED TO CHANGE
# TOT_CLICKS = 4

# # Helper function to check computed homography
# # This will draw dots in a grid by projecting x,y coordinates
# # of tile corners to u,v image coordinates
# def check_homography(image, H, nx, ny, length=TILE_LENGTH):
#   # H should be a 3x3 numpy.array
#   # nx is the number of tiles in the x direction
#   # ny is the number of tiles in the y direction
#   # length is the length of one side of a tile
#   # image is an image array
#   for i in range(nx+1):
#     for j in range(ny+1):
#       xbar = np.array([[i*length],[j*length],[1]])
#       ubar = np.dot(H,xbar).T[0]
#       u = np.int(ubar[0]/ubar[2])
#       v = np.int(ubar[1]/ubar[2])
#       print 'Dot location: ' + str((u,v))
#       cv2.circle(image, (u,v), 5, 0, -1)
#   cv2.imshow('Check Homography', image)

# # Create a CvBridge to convert ROS messages to OpenCV images
# bridge = CvBridge()

# # Converts a ROS Image message to a NumPy array to be displayed by OpenCV
# def ros_to_np_img(ros_img_msg):
#   return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

# if __name__ == '__main__':
  
#   # Waits for the image service to become available
#   rospy.wait_for_service('last_image')
  
#   # Initializes the image processing node
#   rospy.init_node('image_processing_node')
  
#   # Creates a function used to call the 
#   # image capture service: ImageSrv is the service type
#   last_image_service = rospy.ServiceProxy('last_image', ImageSrv)

#   # Create an empty list to hold the coordinates of the clicked points
#   points = []

#   # Callback function for 'cv2.SetMouseCallback' adds a clicked point to the
#   # list 'points'
#   def on_mouse_click(event,x,y,flag,param):
#     if(event == cv2.EVENT_LBUTTONUP):
#       point = (x,y)
#       print "Point Captured: " + str(point)
#       points.append(point)

#   while not rospy.is_shutdown():
#     try:
#       # XXX NEEDS TO BE AUTOMATED
#       rospy.wait_for_service('last_image')
#       # Waits for a key input to continue
#       raw_input('Press enter to capture an image:')
#     except KeyboardInterrupt:
#       print 'Break from raw_input'
#       break
    
#     try:
#       # Request the last image from the image service
#       # And extract the ROS Image from the ImageSrv service
#       # Remember that ImageSrv.image_data was
#       # defined to be of type sensor_msgs.msg.Image
#       ros_img_msg = last_image_service().image_data

#       # Convert the ROS message to a NumPy image
#       np_image = ros_to_np_img(ros_img_msg)

#       # Display the CV Image
#       cv2.imshow("CV Image", np_image)

#       # Tell OpenCV that it should call 'on_mouse_click' when the user
#       # clicks the window. This will add clicked points to our list
#       cv2.setMouseCallback("CV Image", on_mouse_click, param=1)

#       # Zero out list each time we have a new image
#       points = []

#       # Loop until the user has clicked enough points
#       while len(points) < TOT_CLICKS:
#         if rospy.is_shutdown():
#           raise KeyboardInterrupt
#         cv2.waitKey(10)

#       # Convert the Python list of points to a NumPy array of the form
#       #   | u1 u2 u3 u4 |
#       #   | v1 v2 v3 v4 |
#       uv = np.array(points).T

# # === YOUR CODE HERE ===========================================================
      
#       # This is placeholder code that will draw a 4 by 3 grid in the corner of
#       # the image
#       nx = 4
#       ny = 3
#       # Define ground coordinates
#       g_coords = np.array([[0, 0], [nx*TILE_LENGTH, 0], [nx*TILE_LENGTH, ny*TILE_LENGTH], [0, ny*TILE_LENGTH]])

#       A = np.zeros([8, 8])
#       b = np.array([])
#       i = 0
#       for coord in g_coords:
#           u = uv[0, i]
#           v = uv[1, i]
#           x = coord[0]
#           y = coord[1]
#           # Add u_i, v_i to the b array
#           b = np.append(b, [u, v])
#           # Make the first row to be added to A
#           array1 = np.array([x, y, 1, 0, 0, 0, -u*x, -u*y])
#           # Make the second row to be added to A
#           array2 = np.array([0, 0, 0, x, y, 1, -v*x, -v*y])
#           A[2*i] = array1
#           A[2*i+1] = array2
#           i += 1

#       # Calculate H
#       x = np.append(np.dot(np.linalg.inv(A), b.T), 1)
#       H = x.reshape(3, 3)
# # ==============================================================================
      
#       # Check the produced homography matrix
#       check_homography(np_image, H, nx, ny)


#       # Get more user clicks
#       while len(points) < TOT_CLICKS+2:
#         if rospy.is_shutdown():
#           raise KeyboardInterrupt
#         cv2.waitKey(10)

#       uv = np.array(points);
#       uv = uv[TOT_CLICKS:TOT_CLICKS+1];
#       uv = uv.T;

#       u = uv[0]
#       v = uv[1]

#       Q = np.linalg.inv(H)
#       x = (Q[0, 0]*u + Q[0, 1]*v + Q[0, 2])/(Q[2, 0]*u + Q[2, 1]*v + Q[2, 2])
#       y = (Q[1, 0]*u + Q[1, 1]*v + Q[1, 2])/(Q[2, 0]*u + Q[2, 1]*v + Q[2, 2])

#       print np.absolute(x-y)

#       # Loop until the user presses a key
#       key = -1
#       while key == -1:
#         if rospy.is_shutdown():
#           raise KeyboardInterrupt
#         key = cv2.waitKey(100)
      
#       # When done, get rid of windows and start over
#       # cv2.destroyAllWindows()

#     except KeyboardInterrupt:
#       print 'Keyboard Interrupt, exiting'
#       break

#     # Catch if anything went wrong with the Image Service
#     except rospy.ServiceException, e:
#       print "image_process: Service call failed: %s"%e
    
#   cv2.destroyAllWindows()


