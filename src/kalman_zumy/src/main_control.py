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
        # True if Zumy has arrived at the current goal
        self.at_goal = False
        # True if Zumy has finished sorting all utensils
        self.done = False

    """ Calculate the 4x4 rigid body transform from the
    provided quarternions.

    Keyword arguments:
    trans -- the translation vector [3x1 array]
    rot   -- the rotation vector (in quarternions) [4x1 array]
    """
    def return_rbt(self, trans, rot):
        omega, theta = eqf.quarternion_to_exp(rot)
        return eqf.create_rbt(omega, theta, trans)

    """ Returns zumy heading (angle from x axis, in degrees)
    """ 
    def heading(self):
        zumy_tag = self.ar_tags["zumy"]
        start_tag = self.ar_tags["start"]
        (trans, rot) = self.listener.lookupTransform(zumy_tag, start_tag, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        return (180/np.pi)*euler[2]

    """ Turn the zumy 90 degrees in the appropriate
    direction.
    
    Keyword arguments:
    direction -- "left" or "right" [string]
    """
    def turn(self, direction):
        # Create turn command
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        if direction == "left":
            cmd.angular.z = 0.2
        elif direction == "right":
            cmd.angular.z = -0.2

        done = False
        # Get current angle of Zumy relative to start
        current_angle = self.heading()
        while not rospy.is_shutdown() and not done:
            try:
                # Get new angle of Zumy relative to start
                new_angle = self.heading()
                    if abs(current_angle-new_angle) > 85:
                        done = True
                    self.stop()
                    else:
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
        (trans, rot) = self.listener.lookupTransform(zumy_tag, start_tag, rospy.Time(0))
        xTrans = trans[0]
        yTrans = trans[1]
        return [xTrans, yTrans]

    """ Move the Zumy along the x- or y-axis for
    a given distance

    Keyword arguments:
    amount -- amount to move by [0.1 -> 10cm]
    """
    def forward(self, amount):
      cmd = Twist()
      cmd.linear.y = 0
      cmd.linear.z = 0
      cmd.angular.x = 0
      cmd.angular.y = 0
      cmd.angular.z = 0

      if amount > 0:
        cmd.linear.x = 0.1
      else:
        cmd.linear.x = -0.1
        
      done = False
      #Get current position of Zumy relative to start(origin)
      coor0 = self.xyPos()
      while not rospy.is_shutdown() and not done:
        try:
            coor1 = self.xyPos()
            dist = np.sqrt((coor1[0]-coor0[0])**2 + (coor1[1]-coor0[1])**2)
            print dist
            if (dist > amount):
              done = True
              self.stop()
            else:
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
    # def moveToGoal(self):
    #     while not rospy.is_shutdown() and not self.at_goal:
        #     try:
        #         ar_tags = self.ar_tags
        #         zumy_tag = ar_tags["zumy"]
        #         goal_tag = ar_tags[self.current_goal]
        #         (trans, rot) = self.listener
        #                            .lookupTransform(zumy_tag,
        #                                             goal_tag,
        #                                             rospy.Time(0))
        #     except:
        #         continue
        #     rbt = self.return_rbt(trans, rot)
        #     yTrans = -1*rbt[0, 3]
        #     # Check if the y translation is less than some
        #     # threshold
        #     if abs(yTrans <= 0.15):
        #         done = True
        #     else:
        #         cmd = Twist()
        #         cmd.linear.x = 0
        #         cmd.linear.y = 0
        #         cmd.linear.z = 0
        #         cmd.angular.x = 0
        #         cmd.angular.y = 0
        #         cmd.angular.z = 0.2
        #         self.zumy_vel.publish(cmd)
        #     self.rate.sleep()
        # return

    # def returnToStart(self):
    #     while not rospy.is_shutdown() and not self.at_goal:
    #         self.rate.sleep()

    #     self.path = []
    #     self.current_goal = None
    #     return

    """ Main node execution function.
    """
    def run(self):
        while not rospy.is_shutdown() and not self.done:
            try:
                self.wait_rate.sleep()
                direction = raw_input('How much should I go')
                direction = raw_input('Sorry, what was that?')
                self.forward(float(direction))
                self.rate.sleep()
            except KeyboardInterrupt:
                self.stop()
                break
        # Set up a function to call the utensil detection service. The
        # service is set up in image_process.py and uses the UtensilSrv
        # service type.
        # rospy.wait_for_service("utensil_type")
        # rospy.wait_for_service("path_planning")
        # get_utensil_type = rospy.ServiceProxy("utensil_type", UtensilSrv)
        # plan_path = rospy.ServiceProxy("plan_path", PathPlanningSrv)
        # while not rospy.is_shutdown() and not self.done:
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
                # self.path = plan_path(self.origin, self.current_goal).path_points
                # self.moveToGoal()
                # self.returnToStart()
            # else:
                # The Zumy has finished sorting all utensils.
                # self.done = True
            # self.rate.sleep()

if __name__ == "__main__":
    # if len(sys.argv) < 7:
    #     print("Use: main_control.py [ zumy name ] [ AR zumy tag # ] [ AR start tag #] [ AR fork tag # ] [ AR knife tag # ] [ AR spoon tag # ]")
    #     sys.exit()
    if len(sys.argv) < 4:
        print("Use: main_control.py [ zumy name ] [ AR zumy tag # ] [ AR start tag #] ")
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags["zumy"]  = "ar_marker_" + sys.argv[2]
    ar_tags["start"] = "ar_marker_" + sys.argv[3]
    # ar_tags["fork"]  = "ar_marker_" + sys.argv[4]
    # ar_tags["knife"] = "ar_marker_" + sys.argv[5]
    # ar_tags["spoon"] = "ar_marker_" + sys.argv[6]
    node = MainControl(zumy_name, ar_tags)
    node.run()
