#!/usr/bin/env python3
import argparse

import rospy
from std_msgs.msg import String
from psr_parte8_ex3.msg import Dog

def callback(msg):
    rospy.loginfo(" Received message: " + msg.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    ap = argparse.ArgumentParser()
    ap.add_argument('-tn', '--topic_Name', type=str, required=True,
                    help='Name of the topic to publish')
    args = vars(ap.parse_args())

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(args['topic_Name'], String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
