#!/usr/bin/env python3
# license removed for brevity
import rospy
import visualization_msgs.msg
import random
from std_msgs.msg import String, ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3


def main():
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('marker', MarkerArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    count=0

    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        count += 0.1
        marker = Marker()

        marker.header = Header(stamp=rospy.Time.now(), frame_id='world')

        marker.type = Marker.CUBE
        marker.ns = 'Cube'

        point = Point(x=0, y=0, z=0)
        quaternion = Quaternion(x=0, y=0, z=0, w=1)
        pose = Pose(position=point, orientation=quaternion)
        marker.pose = pose

        scale = Vector3(x=1, y=1, z=1)
        marker.scale = scale

        color = ColorRGBA(r=255, g=0, b=0, a=1)
        marker.color = color

        marker_array.markers.append(marker)

        marker2 = Marker()

        marker2.header = Header(stamp=rospy.Time.now() , frame_id='world')

        marker2.type = Marker.SPHERE_LIST
        marker2.ns = 'Sphere'

        point = Point(x=0, y=0, z=0)
        quaternion = Quaternion(x=0, y=0, z=0, w=1)
        pose = Pose(position=point, orientation=quaternion)
        marker2.pose = pose

        scale = Vector3(x=1, y=1, z=1)
        marker2.scale = scale

        color = ColorRGBA(r=1, g=0, b=1, a=1)
        marker2.color = color


        marker_array.markers.append(marker2)


        marker2.points=[]
        for i in range(0,10):
            x = random.randint(-3, 3)
            y = random.randint(-3, 3)
            z = random.randint(-3, 3)
            marker2.points.append(Point(x=x, y=y, z=z))

        #pub.publish(marker)
        #pub.publish(marker2)
        pub.publish(marker_array)

        rospy.loginfo("Publishing marker")
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
