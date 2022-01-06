#!/usr/bin/env python3
# license removed for brevity

import argparse
import rospy
from std_msgs.msg import String
from psr_parte8_ex3.msg import Dog

def talker():
    # Process arguments

    pub = rospy.Publisher("chatter", Dog, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        dog = Dog()
        dog.name = 'Boby'
        dog.age = 77
        dog.color  = 'brown'
        dog.brothers.append('Rosita')
        rospy.loginfo('Sending Dog')
        pub.publish(dog)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
