#!/usr/bin/env python3
import random

import rospy
import math
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

publisher = rospy.Publisher('/markers', MarkerArray, queue_size=1)


def createMarker():
    # create marker
    marker = Marker()
    marker.header.frame_id = "left_laser"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.pose.orientation.w = 1.0  # otherwise quaternion is not normalized
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = random.random()
    marker.color.g = random.random()
    marker.color.b = random.random()
    return marker

def callback_message_received(msg):
    rospy.loginfo('Received laser scan message')

    x_prev, y_prev = 1000, 1000
    dist_threshold = 1
    marker_array = MarkerArray()

    marker = createMarker()
    marker.points = []

    z = 0
    for idx, range in enumerate(msg.ranges):

        if range < 0.1:
            continue

        theta = msg.angle_min + msg.angle_increment * idx
        x = range * math.cos(theta)
        y = range * math.sin(theta)

        # should I create a new group?
        dist = math.sqrt((x_prev-x)**2 + (y_prev-y)**2)
        if dist > dist_threshold:  # new group
            marker = createMarker()
            idx_group = len(marker_array.markers)
            marker.id = idx_group
            marker.points = []
            marker_array.markers.append(marker)

        last_marker = marker_array.markers[-1]
        last_marker.points.append(Point(x=x, y=y, z=z))

        x_prev = x
        y_prev = y

    publisher.publish(marker_array)  # publish (will automatically convert from point_cloud2 to Pointcloud2 message)
    rospy.loginfo('Published marker array msg')

def main():
    rospy.init_node('lidar_subscriber', anonymous=True)

    rospy.Subscriber('/left_laser/laserscan', LaserScan, callback_message_received)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
