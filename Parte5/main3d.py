#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import json

import cv2
import numpy as np
from functools import partial

# ---------------------------------------------------
# Global Variables
# ---------------------------------------------------
min_BH = 0
max_BH = 255
min_GS = 0
max_GS = 255
min_RV = 0
max_RV = 255

def minonTrackbar_B_H(threshold):

    global min_BH
    min_BH = threshold
    print('O limite minimo definido para o B/H é: '+str(min_BH))

def maxonTrackbar_B_H(threshold):

    global max_BH
    max_BH = threshold
    print('O limite maximo definido para o B/H é: '+str(max_BH))

def minonTrackbar_G_S(threshold):

    global min_GS
    min_GS = threshold
    print('O limite minimo definido para o G/S é: '+str(min_GS))

def maxonTrackbar_G_S(threshold):

    global max_GS
    max_GS = threshold
    print('O limite maximo definido para o G/S é: '+str(max_GS))

def minonTrackbar_R_V(threshold):

    global min_RV
    min_RV = threshold
    print('O limite minimo definido para o R/V é: '+str(min_RV))

def maxonTrackbar_R_V(threshold):

    global max_RV
    max_RV = threshold
    print('O limite maximo definido para o R/V é: '+str(max_RV))



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
    cv2.createTrackbar('Min B/H', window_name, 0, 255, minonTrackbar_B_H)
    cv2.createTrackbar('Max B/H', window_name, 0, 255, maxonTrackbar_B_H)
    cv2.createTrackbar('Min G/S', window_name, 0, 255, minonTrackbar_G_S)
    cv2.createTrackbar('Max G/S', window_name, 0, 255, maxonTrackbar_G_S)
    cv2.createTrackbar('Min R/V', window_name, 0, 255, minonTrackbar_R_V)
    cv2.createTrackbar('Max R/V', window_name, 0, 255, maxonTrackbar_R_V)

    # Set the trackbar position to 255 for maximum trackbars
    cv2.setTrackbarPos('Max B/H', window_name, 255)
    cv2.setTrackbarPos('Max G/S', window_name, 255)
    cv2.setTrackbarPos('Max R/V', window_name, 255)

    # Initialize binarized image with threshold equal to 0 or 255
    minonTrackbar_B_H(0)
    maxonTrackbar_B_H(255)
    minonTrackbar_G_S(0)
    maxonTrackbar_G_S(255)
    minonTrackbar_R_V(0)
    maxonTrackbar_R_V(255)

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

        key=cv2.waitKey(10)  # wait a key

        # Stop the cycle if 'q' is pressed and save the ranges dictionary
        if key == ord('q'):
            print('Letter q (quit) pressed, exiting the program and saving ranges')

            file_name = 'limits.json'
            with open(file_name, 'w') as file_handle:
                print("writing dictionary 'ranges' to file " + file_name)
                json.dump(ranges, file_handle)  # ranges is the dictionary

            break


if __name__ == "__main__":
    main()
