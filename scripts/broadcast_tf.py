#!/usr/bin/env python  
import roslib
import rospy

import tf

def broadcast():
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "camera_link",
                     "map")

    rospy.loginfo('Broadcast tf')

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')

    while not rospy.is_shutdown():
        broadcast()
        rospy.sleep(0.01)
