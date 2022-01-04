#!/usr/bin/python3
import string

import cv2
import argparse
import numpy as np
from numpy.ma import copy


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

    ranges = {'b': {'min': 0, 'max': 50},
              'g': {'min': 85, 'max': 110},
              'r': {'min': 0, 'max': 50}}

    mins = np.array([ranges['b']['min'], ranges['g']['min'], ranges['r']['min']])
    maxs = np.array([ranges['b']['max'], ranges['g']['max'], ranges['r']['max']])
    mask = cv2.inRange(image_rgb, mins, maxs)
    #Converting from numpy from uint8 to bool
    mask = mask.astype(np.bool)

    image_processed = copy.copy(image_rgb)




    # Visualization
    cv2.imshow('Original', image_rgb)  # Display the im
    cv2.imshow('mask', mask.astype(np.uint8)*255)  # Display the image
    cv2.imshow('Processed', image_processed)  # Display the im
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
