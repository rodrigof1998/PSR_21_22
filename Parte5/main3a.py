#!/usr/bin/python3
import functools
import string

import argparse
from functools import partial
import cv2
import numpy as np
from numpy.ma import copy
import copy
from copy import deepcopy
#include "opencv2/core.hpp"

# Global variables
window_name = 'window - Ex3a'
image_gray = None

def onTrackbar(minimum,maximum):
    global image_gray

    #maximum = cv2.getTrackbarPos('Maximum:', window_name)
    #minimum = cv2.getTrackbarPos('Minimum:', window_name)
    #canny_thresh = cv.getTrackbarPos('canny_thresh:', source_window)
    print(maximum)
    print(minimum)

    maskm = cv2.inRange(image_gray, 0, minimum)
    maskm = ~maskm
    maskm = maskm.astype(np.bool)
    maskM = cv2.inRange(image_gray, maximum, 255)
    maskM = ~maskM
    maskM = maskM.astype(np.bool)
    image_gray_tmp= copy.copy(image_gray)
    image_gray_tmp[maskm]= 0
    image_gray_tmp[maskM]= 255
    cv2.imshow('maskm', maskm.astype(np.uint8)*255)  # Display the image
    cv2.imshow('maskM', maskM.astype(np.uint8)*255)  # Display the image
    cv2.imshow(window_name,image_gray_tmp)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--image', type=str, required=True,help='Full path to image file.')
    args = vars(parser.parse_args())

    image = cv2.imread(args['image'], cv2.IMREAD_COLOR)  # Load an image
    global image_gray # use global var
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # convert bgr to gray image (single channel)
    cv2.namedWindow(window_name)
    #cv2.imshow(window_name,image_gray)
    global max
    global min
    onTrackbarMin = functools.partial(onTrackbar,maximum=20)
    onTrackbarMax = functools.partial(onTrackbar,minimum=20)

    cv2.createTrackbar('Minimum', window_name, 0, 255, onTrackbarMin)
    cv2.createTrackbar('Maximum', window_name, 0, 255, onTrackbarMax)

    #x = cv2.getTrackbarPos('Maximum:', window_name)
    #print(x)
    # add code to create the trackbar ...
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
