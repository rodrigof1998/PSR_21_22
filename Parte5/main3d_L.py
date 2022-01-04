#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import json
import cv2
import numpy as np
from functools import partial

# --------------------------------------------------
# A simple python script to load and read an image using OpenCV
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# ---------------------------------------------------
# Global Variables
# ---------------------------------------------------
min_BH = 0
max_BH = 255
min_GS = 0
max_GS = 255
min_RV = 0
max_RV = 255


# Define functions of trackbars
def minOnTrackbarBH(threshold):
    """
    Trackbar for the minimum threshold for the Blue (RGB) or Hue (HSV) channel
    :param threshold: the threshold for the channel in question. Datatype: int
    """
    global min_BH
    min_BH = threshold
    print('Selected threshold ' + str(min_BH) + ' for limit min B/H')


def maxOnTrackbarBH(threshold):
    """
    Trackbar for the maximum threshold for the Blue (RGB) or Hue (HSV) channel
    :param threshold: the threshold for the channel in question. Datatype: int
    """
    global max_BH
    max_BH = threshold
    print('Selected threshold ' + str(max_BH) + ' for limit max B/H')


def minOnTrackbarGS(threshold):
    """
    Trackbar for the minimum threshold for the Green (RGB) or Saturation (HSV) channel
    :param threshold: the threshold for the channel in question. Datatype: int
    """
    global min_GS
    min_GS = threshold
    print('Selected threshold ' + str(min_GS) + ' for limit min G/S')


def maxOnTrackbarGS(threshold):
    """
    Trackbar for the maximum threshold for the Green (RGB) or Saturation (HSV) channel
    :param threshold: the threshold for the channel in question. Datatype: int
    """
    global max_GS
    max_GS = threshold
    print('Selected threshold ' + str(max_GS) + ' for limit max G/S')


def minOnTrackbarRV(threshold):
    """
    Trackbar for the minimum threshold for the Red (RGB) or Value (HSV) channel
    :param threshold: the threshold for the channel in question. Datatype: int
    """
    global min_RV
    min_RV = threshold
    print('Selected threshold ' + str(min_RV) + ' for limit min R/V')


def maxOnTrackbarRV(threshold):
    """
    Trackbar for the maximum threshold for the Red (RGB) or Value (HSV) channel
    :param threshold: the threshold for the channel in question. Datatype: int
    """
    global max_RV
    max_RV = threshold
    print('Selected threshold ' + str(max_RV) + ' for limit max R/V')


# function which will be called on mouse input
def onMouse(action, x, y, flags, param):
    """
    Function called by cv2.setMouseCallback to print the coordinates of where you clicked in the image
    :param action: to click a button in the mouse, specifically the left button down
    :param x: the x coordinate of the image where you clicked
    :param y: the y coordinate of the image where you clicked
    :param flags: Not used, but needed to the function works
    :param param: Not used, but needed to the function works
    """
    if action == cv2.EVENT_LBUTTONDOWN:
        print('The coordinates of where you clicked are: x = ' + str(x) + ' , y = ' + str(y))


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, help="Input full path image filename")
    ap.add_argument('-hsv', '--hsv', action='store_true', help="Use HSV image or not. If not is RGB image")
    args = vars(ap.parse_args())

    # Load image with given filename and show
    image_filename = args['filename']
    image_rgb = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image_rgb)  # Display the image

    # Convert image to HSV color space if wanted
    if args['hsv']:
        image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2HSV)

    # Create window first
    window_name = 'Segmented'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Create a trackbar to control the threshold of the binarization
    cv2.createTrackbar('min B/H', window_name, 0, 255, minOnTrackbarBH)
    cv2.createTrackbar('max B/H', window_name, 0, 255, maxOnTrackbarBH)
    cv2.createTrackbar('min G/S', window_name, 0, 255, minOnTrackbarGS)
    cv2.createTrackbar('max G/S', window_name, 0, 255, maxOnTrackbarGS)
    cv2.createTrackbar('min R/V', window_name, 0, 255, minOnTrackbarRV)
    cv2.createTrackbar('max R/V', window_name, 0, 255, maxOnTrackbarRV)

    # Set the trackbar position to 255 for maximum trackbars
    cv2.setTrackbarPos('max B/H', window_name, 255)
    cv2.setTrackbarPos('max G/S', window_name, 255)
    cv2.setTrackbarPos('max R/V', window_name, 255)

    # Initialize trackbars
    minOnTrackbarBH(0)
    maxOnTrackbarBH(255)
    minOnTrackbarGS(0)
    maxOnTrackbarGS(255)
    minOnTrackbarRV(0)
    maxOnTrackbarRV(255)

    # While cycle to update the values of the trackbar at the time
    while True:
        # Get ranges for each channel from trackbar
        if not args['hsv']:
            ranges = {'B': {'min': min_BH, 'max': max_BH},
                      'G': {'min': min_GS, 'max': max_GS},
                      'R': {'min': min_RV, 'max': max_RV}}
        else:
            ranges = {'H': {'min': min_BH, 'max': max_BH},
                      'S': {'min': min_GS, 'max': max_GS},
                      'V': {'min': min_RV, 'max': max_RV}}

        # Convert the dict structure created before to numpy arrays, because opencv uses it.
        if not args['hsv']:
            mins = np.array([ranges['B']['min'], ranges['G']['min'], ranges['R']['min']])
            maxs = np.array([ranges['B']['max'], ranges['G']['max'], ranges['R']['max']])
        else:
            mins = np.array([ranges['H']['min'], ranges['S']['min'], ranges['V']['min']])
            maxs = np.array([ranges['H']['max'], ranges['S']['max'], ranges['V']['max']])

        # Create mask using cv2.inRange. The output is still in uint8
        if not args['hsv']:
            segmented = cv2.inRange(image_rgb, mins, maxs)
        else:
            segmented = cv2.inRange(image_hsv, mins, maxs)

        # Show segmented image
        cv2.imshow(window_name, segmented)  # Display the image

        # Get the coordinates of where you clicked in the image
        cv2.setMouseCallback(window_name, onMouse)

        key = cv2.waitKey(10)  # Wait a key to stop the program, the waitKey is only waiting 10 ms.

        # Stop the cycle if 'q' is pressed and save the ranges dictionary
        if key == ord('q'):
            print('Letter q (quit) pressed, exiting the program and saving ranges')

            file_name = 'ranges.json'
            with open(file_name, 'w') as file_handle:
                print("writing dictionary 'ranges' to file " + file_name)
                json.dump(ranges, file_handle)  # ranges is the dictionary

            break


if __name__ == "__main__":
    main()
