#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from psr_parte8_ex4.msg import Dog


def talker():
    pub = rospy.Publisher('chatter', Dog, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Ola mundo %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
