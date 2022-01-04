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
    image_gray = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2GRAY)

    image_b, image_g, image_r = cv2.split(image_rgb)

    _, image_b_processed = cv2.threshold(image_b, 50, 255, cv2.THRESH_BINARY)
    _, image_g_processed = cv2.threshold(image_g, 100, 255, cv2.THRESH_BINARY)
    _, image_r_processed = cv2.threshold(image_r, 150, 255, cv2.THRESH_BINARY)

    new_image_rgb = cv2.merge((image_b_processed, image_g_processed, image_r_processed))

    # Visualization
    cv2.imshow('Original', image_rgb)  # Display the image
    cv2.imshow('Gray', image_gray)  # Display the image
    cv2.imshow('processed', new_image_rgb)  # Display the image
    #cv2.imshow('B', image_b)  # Display the image
    #cv2.imshow('G', image_g)  # Display the image
    #cv2.imshow('R', image_r)  # Display the image
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
