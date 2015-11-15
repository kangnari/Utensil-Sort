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

def update_kalman(ar_tags):
    print "started update"
    rospy.wait_for_service('innovation')
    update = rospy.ServiceProxy('innovation', NuSrv)
    listener = tf.TransformListener()
    while True:
        try:
            try:
               (trans, rot) = listener.lookupTransform(ar_tags['ar1'], ar_tags['arZ'], rospy.Time(0))
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
    rospy.init_node('watch_ar_tags')
    if len(sys.argv) < 4:
        print('Use: follow_ar_tag_manual.py [ zumy name ] [ AR tag number for goal] [ AR tag number for Zumy] ')
        sys.exit()
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
    ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]

    #follow_ar_tag(zumy=zumy_name, ar_tags=ar_tags)
    update_kalman(ar_tags)
    rospy.spin()
