#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Quaternion, Transform, Vector3, Twist
import exp_quat_func as eqf
from kalman_zumy.srv import *

listener = None

def return_rbt(trans, rot):
    """
    Prints out the 4x4 rigid body transformation matrix from quaternions

    Input:
        (3,) array - translation ector
        (4,) array - rotation vector in quaternions
    """

    #YOUR CODE HERE
    omega, theta = eqf.quaternion_to_exp(rot)
    rbt = eqf.create_rbt(omega, theta, trans)
    return rbt

def compute_twist(rbt):
    """
    Computes the corresponding twist for the rigid body transform

    Input:
        rbt - (4,4) ndarray 

    Output:
        v - (3,) array
        w - (3,) array
    """
    #YOUR CODE HERE
    omega, theta = eqf.find_omega_theta(rbt)
    v = eqf.find_v(omega, theta, rbt[:3, 3])
    return (v,omega)

def pointToAR(zumy, ar_tags, zumy_vel, listener):
    
    print "Pointing toward AR-TAG"
    error_old = None
    # Pointing towards AR-TAG
    done = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not done:
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
        except:
            continue
        # YOUR CODE HERE
        #  orient the Zumy to face the tag

	rbt = return_rbt(trans=trans, rot=rot)
        yTrans = -1*rbt[0, 3]
        print yTrans
        if abs(yTrans) <= 0.15:
            print "Done rotating"
            done = True
	print "Rotating"
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0.15
        zumy_vel.publish(cmd)
	rate.sleep()
    return

def moveToAR(zumy, ar_tags, zumy_vel, listener):
    print "MOVING TO AR TAG"
    error_old = None
    # move towards the AR tag
    done = False
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and not done:
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
        except:
            continue
        # YOUR CODE HERE
        #  drive the zumy forward (forward is the X direction) close to the AR tag
        #  you can also fix deviations on the trajectory by adjusting the rotation
        #  as it moves forward
        rbt = return_rbt(trans=trans, rot=rot)
	
	yTrans = -1*rbt[0, 3]
        print yTrans
        if abs(yTrans) > 0.15:	
		break

        xTrans = -1*rbt[1, 3]
	print xTrans
        moveAmt = 0.1
        if xTrans < 0:
            moveAmt = -0.1
        if abs(xTrans) <= 0.13:
            done = True
	    break
	print "Translating"
        cmd = Twist()
        cmd.linear.x = moveAmt
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        zumy_vel.publish(cmd)
	rate.sleep()
    return

def follow_ar_tag(zumy, ar_tags):

    listener = tf.TransformListener()
    zumy_vel = rospy.Publisher('/%s/cmd_vel' % zumy, Twist, queue_size=2)
    done = False
    while not done:
	try:
            (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
        except:
            continue
	rbt = return_rbt(trans=trans, rot=rot)
	
	xTrans = -1*rbt[1, 3]
	yTrans = -1*rbt[0, 3]
        if abs(xTrans) <= 0.15 and abs(yTrans) <= 0.1:
            done = True
	    break
	pointToAR(zumy, ar_tags, zumy_vel, listener)
	moveToAR(zumy, ar_tags, zumy_vel, listener)

    # Stop the zumy
    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0
    zumy_vel.publish(cmd)
    print("DONE")

def update_kalman(ar_tags):
    print "started update"
    rospy.wait_for_service('innovation')
    update = rospy.ServiceProxy('innovation', NuSrv)
    listener = tf.TransformListener()
    while True:
        try:
            try:
               (trans, rot) = listener.lookupTransform(ar_tags['arZ'], ar_tags['ar1'], rospy.Time(0))
            except:
	       print "Couldn't look up transform"
               continue
            lin = Vector3()
            quat = Quaternion()
            lin.x = trans[0]
            lin.y = trans[1]
            lin.z = trans[2]
            quat.x = rot[0]
            quat.y = rot[1]
            quat.z = rot[2]
            quat.w = rot[3]
            transform = Transform()
            transform.translation = lin
            transform.rotation = quat
            test = update(transform, ar_tags['ar1'])
            print "Service call succeeded"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__=='__main__':
    rospy.init_node('follow_ar_tag_manual')
    if len(sys.argv) < 4:
        print('Use: follow_ar_tag_manual.py [ zumy name ] [ AR tag number for goal] [ AR tag number for Zumy] ')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    #update_kalman(ar_tags)
    rospy.spin()
