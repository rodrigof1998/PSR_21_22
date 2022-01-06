#!/usr/bin/env python3
# license removed for brevity

import argparse

import colorama
import rospy
from std_msgs.msg import String
from psr_parte8_ex3.msg import Dog

def talker():
    # Process arguments
    rospy.init_node('publisher', anonymous=True)
    pub = rospy.Publisher("chatter", Dog, queue_size=10)

    # read private parameter
    frequency = rospy.get_param("~frequency", default=1)

    rate = rospy.Rate(frequency) # 10hz

    while not rospy.is_shutdown():
        # read global parameter
        highlight_text_color = rospy.get_param("/highlight_text_color")

        dog = Dog()
        dog.name = 'Boby'
        dog.age = 77
        dog.color  = 'brown'
        dog.brothers.append('Rosita')
        rospy.loginfo('Sending Dog with name ' +
                      getattr(colorama.Fore, highlight_text_color) + str(dog.name) +
                      colorama.Style.RESET_ALL)
        pub.publish(dog)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
