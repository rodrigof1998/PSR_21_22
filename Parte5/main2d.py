#!/usr/bin/python3
import string

import cv2
import argparse
import numpy as np

def main():

    # Process arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-path', '--file_location', type=str, required=True,
                    help='File directory1')
    args = vars(ap.parse_args())

    # Load image
    image_filename = args['file_location']
    image_rgb = cv2.imread(image_filename, cv2.IMREAD_COLOR) # Load an image

    # Process image

    ranges = {'b': {'min': 0, 'max': 100},
              'g': {'min': 85, 'max': 256},
              'r': {'min': 0, 'max': 100}}

    mins = np.array([ranges['b']['min'], ranges['g']['min'], ranges['r']['min']])
    maxs = np.array([ranges['b']['max'], ranges['g']['max'], ranges['r']['max']])
    image_processed = cv2.inRange(image_rgb, mins, maxs)

    # Visualization
    cv2.imshow('Original', image_rgb)  # Display the im
    cv2.imshow('processed', image_processed)  # Display the image
    #cv2.imshow('B', image_b)  # Display the image
    #cv2.imshow('G', image_g)  # Display the image
    #cv2.imshow('R', image_r)  # Display the image
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
