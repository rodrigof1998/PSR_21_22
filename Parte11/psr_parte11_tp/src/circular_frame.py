#!/usr/bin/env python3
import math

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


if __name__ == '__main__':
    rospy.init_node('circular_frame')

    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(100)
    alfa = 0
    rho = 3
    while not rospy.is_shutdown():
        alfa +=0.01
        if alfa > 2* math.pi:
            alfa = 0
        # Create and populate my transformation
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'parent'
        t.child_frame_id = 'child'
        t.transform.translation.x = rho * math.cos(alfa)
        t.transform.translation.y = rho * math.sin(alfa)
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 10*alfa)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send transformation
        br.sendTransform(t)

        #slepp
        rate.sleep()

    rospy.spin()