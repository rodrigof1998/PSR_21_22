#!/usr/bin/env python3
import cv2
import rospy
import math
import std_msgs.msg
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image


def main():
    # initialize the ros node
    rospy.init_node('image_publisher', anonymous=False)

    # set up the publisher
    publisher = rospy.Publisher('~image', Image, queue_size=1)

    # video capture setup
    capture = cv2.VideoCapture(0)
    window_name = "OpenCv window"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    rate = rospy.Rate(15)

    while True:     # infinite loop of image reading
        _, image = capture.read()   # get an image from camera
        cv2.imshow(window_name, image)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        publisher.publish(image_message)

        if cv2.waitKey(1) == ord('q'):
            break

        rate.sleep()

    # When everything is done, release the capture
    capture.release()
    cv2.destroyAllWindows()

    rospy.spin()


if __name__ == '__main__':
    main()