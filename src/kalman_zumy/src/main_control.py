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
        self.zumy_vel = rospy.Publisher('/%s/cmd_vel' % zumy, Twist, queue_size=2)
        # Set the zumy name
        self.mname = mname
        # Set the provided ar tags
        self.ar_tags = ar_tags
        # Set the current goal for the zumy
        self.current_goal = self.ar_tags["start"]
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

    """ Given a path output from the path planning service node,
    move the Zumy point by point until it has reached its current
    goal.
    """
    def moveToGoal(self):
        done = False
        # while not rospy.is_shutdown() and not done:
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

    """ Main node execution function.
    """
    def run(self):
        # Set up a function to call the utensil detection service. The
        # service is set up in image_process.py and uses the UtensilSrv
        # service type.
        # rospy.wait_for_service("utensil_type")
        # rospy.wait_for_service("path_planning")
        # get_utensil_type = rospy.ServiceProxy("utensil_type", UtensilSrv)
        # plan_path = rospy.ServiceProxy("", PathPlanSrv)
        while not rospy.is_shutdown() and not self.done:
            if self.at_goal:
                if self.current_goal == self.ar_tags["start"]
                    # XXX FUNCTION HERE TO DROP UTENSIL
                    # Pause until the next utensil has dropped.
                    self.wait_rate.sleep()
                    # Get the type of the utensil.
                    # utensil = get_utensil_type().utensil_type
                    utensil = raw_input('Input current goal for zumy:')
                    if utensil:
                        # Set the new goal for the Zumy.
                        self.current_goal = self.ar_tags[utensil]
                    else:
                        # The Zumy has finished sorting all utensils.
                        self.done = True
                else:
                    # Zumy has just finished moving its utensil to
                    # the correct pile.
                    self.current_goal = self.ar_tags["start"]
                    # Set the path the Zumy should follow
                    # self.path = plan_path(self.current_goal).path_points
            else:
                # Zumy needs to move towards the goal
                # Call path planning service
                self.moveToGoal()
            self.rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 7:
        print("Use: main_control.py [ zumy name ] [ AR zumy tag # ] [ AR start tag #] [ AR fork tag # ] [ AR knife tag # ] [ AR spoon tag # ]")
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags["zumy"]  = "ar_marker_" + sys.argv[2]
    ar_tags["start"] = "ar_marker_" + sys.argv[3]
    ar_tags["fork"]  = "ar_marker_" + sys.argv[4]
    ar_tags["knife"] = "ar_marker_" + sys.argv[5]
    ar_tags["spoon"] = "ar_marker_" + sys.argv[6]
    node = MainControl(zumy_name, ar_tags)
    node.run()
