#!/usr/bin/env python3
import math

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def main():
    rospy.init_node('circular_frame')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    distance_to_parent = rospy.get_param("~distance_to_parent")
    period = rospy.get_param("~period")

    rate = rospy.Rate(100)
    alfa = 0

    while not rospy.is_shutdown():
        alfa += (1/period)/100
        if alfa > 2 * math.pi:
            alfa = 0

        t.header.stamp = rospy.Time.now()

        # Sun to mercury
        t.header.frame_id = rospy.remap_name('parent')
        t.child_frame_id = rospy.remap_name('child')
        t.transform.translation.x = distance_to_parent * math.cos(alfa)
        t.transform.translation.y = distance_to_parent * math.sin(alfa)
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1
        br.sendTransform(t)          # Send transformation

        #slepp
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass