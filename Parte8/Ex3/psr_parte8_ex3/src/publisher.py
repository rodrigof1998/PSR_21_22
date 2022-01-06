#!/usr/bin/env python3
# license removed for brevity

import argparse
import rospy
from std_msgs.msg import String
from psr_parte8_ex3.msg import Dog

def talker():
    # Process arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-tn', '--topic_Name', type=str, required=True,
                    help='Name of the topic to publish')
    ap.add_argument('-msg', '--message', type=str, required=True,
                    help='Message to publish')
    ap.add_argument('-f', '--frequency', type=float, required=True,
                    help='Frequency of publish')
    args = vars(ap.parse_args())

    pub = rospy.Publisher(args['topic_Name'], String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(args['frequency']) # 10hz

    while not rospy.is_shutdown():
        text_to_send = "Publishing message: " + args['message'] + " on topic " + args['topic_Name']
        rospy.loginfo(text_to_send)
        pub.publish(args['message'])
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
