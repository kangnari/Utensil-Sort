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
import priority_queue as PriorityQueue

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
        # Set the pixel locations for 
        self.goal_px_locations = {
            'start': [0, 0],
            'fork': [0, 0],
            'knife': [0, 0],
            'spoon': [0, 0]
        }
        # Set the x/y boundary lengths
        self.x_length = x_length
        self.y_length = y_length
        # Set the number of x/y points in the movement grid
        self.nx = num_x
        self.ny = num_y
        # Calculate dx and dy in the grid
        self.dx = x_length/num_x
        self.dy = y_length/num_y
        # Set the color threshold to differentiate obstacles from clear paths
        self.threshold = 123
        # Set up listener for tf.Transform
        self.listener = tf.TransformListener()
        # Homograhy matrix
        self.H = None
        self.debug = debug

        #Create the service
        #rospy.Service('plan_path', PathPlanningSrv, self.getPath)
        
    # for debugging convert from string to int
    def check_homography(self):
        for i in range(self.nx + 1):
            for j in range(self.ny + 1):
                xbar = np.array([[i*self.dx], [j*self.dy], [1]])
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
        A = np.zeros((8, 8))
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
    
    Keyword arguments:
    request -- the request for the path. Indicates the goal of interest
    """
    def getPath(self, request):
        # Set up a function to call the image capture service. The
        # service is set up in camera_srv.py and uses the ImageSrv
        # service type.
        last_image_service = rospy.ServiceProxy('last_image_global', ImageSrv)

        # Convert image from usb_global_cam to a NumPy image
        image = self.ros_to_np_img(last_image_service().image_data)
        # Convert image to grayscale and set as last_image
        self.last_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        grid = np.zeros((self.ny, self.nx))

        # Get origin and destination as pixel points
        destination_px = self.goal_px_locations[request.destination]
        # Create the obstacle grid
        grid = self.createGrid()
        
        if debug:
            plt.figure()
            plt.imshow(grid.astype(float))
            plt.show()
            cv2.waitKey(100)
            plt.close("all")
        # Calculate the lowest cost paths
        prev = self.Dijkstra(grid, [0, 0])
        
        destX = destination_px[0]
        destY = destination_px[1]

        # Get associated position in grid
        Q = np.linalg.inv(self.H)
        currX = int((Q[0, 0]*destX + Q[0, 1]*destY + Q[0, 2])/(Q[2, 0]*destX + Q[2, 1]*destY + Q[2, 2]) / self.dx)
        currY = int((Q[1, 0]*destX + Q[1, 1]*destY + Q[1, 2])/(Q[2, 0]*destX + Q[2, 1]*destY + Q[2, 2]) / self.dy)
        
        path = [];
        while currX != 0 or currY != 0:
            realX = self.dx * currX; realY = self.dy * currY;
            # If current point is not an obstacle
            if (grid[currX, currY] != 1):
                path.append([realX, realY]) 
            curr = prev[currX, currY, :]
            currX = curr[0]
            currY = curr[1]
        path.append([0, 0])
        return PathPlanningSrvResponse(reversed(path))
    
    """ 
    """
    def createGrid(self):
        grid = np.zeros((self.nx, self.ny))
        for i in range(self.nx):
            for j in range(self.ny):
                xbar = np.array([[i*self.dx], [j*self.dy], [1]])
                ubar = np.dot(self.H, xbar).T[0]
                u = np.int(ubar[0]/ubar[2])
                v = np.int(ubar[1]/ubar[2])
                # Check if the real world point looks like an obstacle or not
                
                # Obstacle = 1
                if (self.last_image[v, u] < self.threshold):
                   grid[i, j] = 1
                # Safe = 0
                else:
                   grid[i, j] = 0
        return grid
                  
    """ Returns a matrix of lowest-cost
    
    Keyword arguments:
    graph -- nx x ny matrix repesenting obstacles on floor [np.matrix]
    source -- starting point for path planning [[x, y]]
    
    Returns:
    dist -- nx x ny matrix with shortest path distances to each point
    [np.matrix]
    
    prev -- nx x ny x 2 matrix containing previous points along shortest path
    [np.matrix]
    """
    def Dijkstra(self, graph, source):
        [X, Y] = graph.shape
        dist = np.zeros(graph.shape)
        prev = np.zeros([X, Y, 2])
        dist[source[0], source[1]] = 0                                  

        Q = PriorityQueue.PriorityQueue()
        numPtsLeft = X*Y;

        for x in range(0, X):
            for y in range(0, Y):
                v = [x, y]
                if x != source[0] or y != source[1]:
                    dist[x, y] = float('Inf')
                    prev[x, y, :] = [-1, -1]
                else:
                    dist[x, y] = 0
                    prev[x, y, :] = [0, 0]
                Q.insert(v, priority=dist[x, y])
            
        while numPtsLeft > 0:
            [priority, u] = Q.pop()
            numPtsLeft -= 1
            neighbors = self.getNeighbors(u)
            for idx in neighbors:
                v = neighbors[idx]
                newDist = dist[u[0], u[1]] + self.getLength(v, graph)
                if newDist < dist[v[0], v[1]]:
                    dist[v[0], v[1]] = newDist
                    prev[v[0], v[1], :] = u
                    Q.insert(v, priority=newDist)
        return prev

    """ Returns a list of points neighboring the
    current point of interest.
    
    Keyword arguments:
    pt -- current point of interest
    """
    def getNeighbors(self, pt):
        neighbors = {}
        x = pt[0]
        y = pt[1]
    
        i = 0
        if x > 0:
            neighbors[i] = [x-1, y]
            i+=1
        if x < self.nx-1:
            neighbors[i] = [x+1, y]
            i+=1
        if y > 0:
            neighbors[i] = [x, y-1]
            i+=1
        if y < self.ny-1:
            neighbors[i] = [x, y+1]
        return neighbors

    """ Determines the cost of moving from the current point
    to a point of interest, based on whether the point of
    interest contains an obstacle.
    
    Keyword arguments:
    v -- potential next point/step [list] [1 x 2]
    graph -- matrix representing the floor (indicates which points
             have obstacles on them [list] [nx x ny]
    """
    def getLength(self, v, graph):
        length = 1
        if graph[v[0], v[1]] == 1:
            length = 1000
        return length
    
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
        corners = ["bottom left boundary corner", "bottom right boundary corner", "top right boundary corner",
                   "top left corner", "fork goal point", "knife goal point", "spoon goal point"]
        # Callback function taken from lab4 code. Adds a clicked
        # point to the points list.
        def on_mouse_click(event, x, y, flag, param):
            if (event == cv2.EVENT_LBUTTONUP):
                point = (x, y)
                points.append(point)
                if len(points) < 7:
                    print "Click on the " + corners[len(points)] + "."
                else:
                    print "Done calibrating image."

        # Comment out next two lines if debugging (CV2 bug)
        cv2.startWindowThread()
        cv2.namedWindow("CV Image")
        # Display the CV Image
        cv2.imshow("CV Image", self.last_image)

        print "Click on the " + corners[0] + " boundary corner."
        cv2.setMouseCallback("CV Image", on_mouse_click, param=1)

        # Have user choose the four boundary corners for
        # Zumy movement 
        while len(points) < len(corners):
            if rospy.is_shutdown():
                raise KeyboardInterrupt
            cv2.waitKey(10)

        # Exit out of picture
        cv2.destroyAllWindows()
        cv2.waitKey(1)

        # | u1 u2 u3 u4 |
        # | v1 v2 v3 v4 |
        self.boundary_corners = np.array(points[:4]).T
        self.goal_px_locations = {
          'start': [points[0][0], points[0][1]],
          'fork': [points[4][0], points[4][1]],
          'knife': [points[5][0], points[5][1]],
          'spoon': [points[6][0], points[6][1]]
        }
        self.create_homography()

        if self.debug:
            self.check_homography()
            path = self.getPath({'destination': 'fork'})
            print path
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
