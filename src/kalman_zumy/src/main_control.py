#!/usr/bin/env python
import rospy
import sys
import tf
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Quaternion, Transform, Vector3, Twist
import exp_quat_func as eqf
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
        rospy.init_node("main_control")
        # Set consistent time intervals for the entire application
        self.hertz = 10
        self.dt = 1/self.hertz
        # Set the rate for sleeping during while loops
        self.rate = rospy.Rate(self.hertz)
        # Set the rate for waiting while utensil is being dropped
        self.wait_rate = rospy.Rate(self.hertz*10)
        # Set up listener for tf.Transform
        self.listener = tf.TransformListener()
        # Set up publisher to set velocity of zumy
        self.zumy_vel = rospy.Publisher('/%s/cmd_vel' % mname, Twist, queue_size=2)
        # Set the zumy name
        self.mname = mname
        # Set the provided ar tags
        self.ar_tags = ar_tags
        # Set the current goal for the zumy
        self.current_goal = None
        # Set the current path for the zumy
        self.path = []
        # True if Zumy is moving toward the origin
        self.to_origin = False
        # True if Zumy has finished sorting all utensils
        self.done = False

    """ Returns zumy heading (angle from x axis, in degrees)
    """ 
    def heading(self):
        zumy_tag = self.ar_tags["zumy"]
        start_tag = self.ar_tags["start"]
        (trans, rot) = self.listener.lookupTransform(start_tag, zumy_tag, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        return (180/np.pi)*euler[2]

    """ Turn the zumy to a certain angle.
    
    Keyword arguments:
    desired_angle -- angle zumy should turn to [number]
    """
    def turn(self, desired_angle):
        # Define parameters for applying variable speed
        min_speed = 0.15
        max_speed = 0.15
        min_angle = 45
        max_angle = 90
        change_speed = (max_speed-min_speed) / (max_angle-min_angle)
        offset = min_speed - change_speed * min_angle

        # Get current angle of Zumy relative to start
        current_angle = self.heading()

        if desired_angle < 0:
            desired_angle = desired_angle + 360
        if current_angle < 0:
            current_angle = current_angle + 360

        # Determine which direction to turn
        if desired_angle < current_angle:
            if current_angle < desired_angle  + 180:
                # Turn right
                direction = -1
            else:
                # Turn left
                direction = 1
        else:
            if desired_angle < current_angle + 180:
                # Turn left
                direction = 1
            else:
                # Turn right
                direction = -1

        # Create turn command
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        done = False

        while not rospy.is_shutdown() and not done:
            try:
                # Get new angle of Zumy relative to start
                new_angle = self.heading()
                if (new_angle < 0):
                    new_angle = new_angle + 360
                diff = abs(desired_angle - new_angle)
                if diff < 5:
                    done = True
                    self.stop()
                else:
                    speed = min(max_speed, max(min_speed, offset+change_speed*diff))
                    cmd.angular.z = direction * speed
                    self.zumy_vel.publish(cmd)
                    self.rate.sleep()
            except KeyboardInterrupt:
                break
        return

    """ Returns the (x,y) position of the Zumy in the coordinate
    frame defined by the origin AR tag
    """
    def xyPos(self):
        zumy_tag = self.ar_tags["zumy"]
        start_tag = self.ar_tags["start"]
        (trans, rot) = self.listener.lookupTransform(start_tag, zumy_tag, rospy.Time(0))
        xTrans = trans[0]*100
        yTrans = trans[1]*100
        return [xTrans, yTrans]

    """ Move the Zumy along the x- or y-axis for
    a given distance

    Keyword arguments:
    amount -- amount to move by [cm]
    """
    def forward(self, amount):
        # Define parameters for applying variable speed
        min_speed = 0.07
        max_speed = 0.1
        min_dist = 5
        max_dist = 20
        change_speed = (max_speed-min_speed) / (max_dist-min_dist)
        offset = min_speed - change_speed * min_dist

        cmd = Twist()
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0

        done = False
        #Get current position of Zumy relative to start(origin)
        coor0 = self.xyPos()
        while not rospy.is_shutdown() and not done:
            try:
                coor1 = self.xyPos()
                dist = np.sqrt((coor1[0]-coor0[0])**2 + (coor1[1]-coor0[1])**2)
                diff = dist-abs(amount)
                if (diff < 1):
                    done = True
                    self.stop()
                else:
                    speed = min(max_speed, max(min_speed, offset+change_speed*diff))
                    cmd.linear.x = np.sign(amount) * speed
                    self.zumy_vel.publish(cmd)
                    self.rate.sleep()
            except KeyboardInterrupt:
                break
        return

    """ Stop Zumy movement.
    """
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.zumy_vel.publish(cmd)
        return

    """ Given a path output from the path planning service node,
    move the Zumy point by point until it has reached its current
    goal.
    """
    """ Given a path output from the path planning service node,
    move the Zumy point by point until it has reached its current
    goal.
    """
    def moveToGoal(self):
        i = 0
        minError = 2;
        path = self.path
        # XXX handle origin case (10cm between centers of ar tags)
        while not rospy.is_shutdown() and i < len(path):
            nextPoint = path[i]
            currPos = self.xyPos()
            currHeading = self.heading()
            
            # Compare x values
            dX = nextPoint[0] - currPos[0]
            
            if self.to_origin:
                # Moving toward origin
                if (abs(dX) > minError):
                    # Move in x direction
                    if(dX > 0):
                        self.turn(0)
                    else:
                        self.turn(180)
                    self.forward(abs(dX))

                # Compuare y values
                currPos = self.xyPos()
                dY = nextPoint[1] - currPos[1]

                if (abs(dY) > minError):
                    # Move in y direction
                    if (dY > 0):
                        self.turn(90)
                    else:
                        self.turn(-90)
                    self.forward(abs(dY))
            else:
                # Moving toward goal
                if (abs(dX) > minError):
                    # Move in x direction
                    if(dX > 0):
                        self.turn(180)
                    else:
                        self.turn(0)
                    self.forward(-1*abs(dX))

                # Compuare y values
                currPos = self.xyPos()
                dY = nextPoint[1] - currPos[1]
                     
                if (abs(dY) > minError):
                    # Move in y direction
                    if (dY > 0):
                        self.turn(-90)
                    else:
                        self.turn(90)
                    self.forward(-1*abs(dY))
            i = i + 1

        # Toggle goal
        self.to_origin = not self.to_origin
        return


    """ Main node execution function.
    """
    def run(self):
        while not rospy.is_shutdown() and not self.done:
            try:
                # XXX testing stuff; remove later
                self.wait_rate.sleep()
                self.path = [[40, 0], [40, 40], [40, 60]]
                direction = raw_input('How much should I go')
                direction = raw_input('Sorry, what was that?')
                # self.forward(float(direction))
                self.moveToGoal()
                self.path = self.path[::-1]
                self.moveToGoal()
                self.rate.sleep()
            except KeyboardInterrupt:
                self.stop()
                break
        # Set up a function to call the utensil detection service. The
        # service is set up in image_process.py and uses the UtensilSrv
        # service type.
        # rospy.wait_for_service("utensil_type")
        rospy.wait_for_service("path_planning")
        # get_utensil_type = rospy.ServiceProxy("utensil_type", UtensilSrv)
        plan_path = rospy.ServiceProxy("plan_path", PathPlanningSrv)
        #while not rospy.is_shutdown() and not self.done:
            # XXX FUNCTION HERE TO DROP UTENSIL
            # Pause until the next utensil has dropped.
            # self.wait_rate.sleep()
            # Get the type of the utensil.
            # utensil = get_utensil_type().utensil_type
            # utensil = raw_input('Input current goal for zumy:')
            # if utensil:
                # Set the new goal for the Zumy.
                # self.current_goal = self.ar_tags[utensil]
                # Zumy needs to move towards the goal
                # Set the path the Zumy should follow
                # self.path = plan_path(self.current_goal).path_points
                # self.moveToGoal()
                # self.returnToStart()
            # else:
                # The Zumy has finished sorting all utensils.
                # self.done = True
            # self.rate.sleep()

if __name__ == "__main__":
    # if len(sys.argv) < 4:
    #     print("Use: main_control.py [ zumy name ] [ AR zumy tag # ] [ AR start tag # ]")
    #     sys.exit()
    if len(sys.argv) < 4:
        print("Use: main_control.py [ zumy name ] [ AR zumy tag # ] [ AR start tag # ] ")
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags["zumy"]  = "ar_marker_" + sys.argv[2]
    ar_tags["start"] = "ar_marker_" + sys.argv[3]
    node = MainControl(zumy_name, ar_tags)
    node.run()
