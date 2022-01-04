#!/usr/bin/python3
import string

import cv2
import argparse
import time

def main():

    # Process arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-path1', '--file_location1', type=str, required=True,
                    help='File directory1')
    ap.add_argument('-path2', '--file_location2', type=str, required=True,
                    help='File directory2')
    args = vars(ap.parse_args())

    # Load image
    image_filename1 = args['file_location1']
    image1 = cv2.imread(image_filename1, cv2.IMREAD_COLOR) # Load an image

    image_filename2 = args['file_location2']
    image2 = cv2.imread(image_filename2, cv2.IMREAD_COLOR) # Load an image
    while(1):
        cv2.imshow('window', image1)  # Display the image
        cv2.waitKey(3000) # wait for a key press before proceeding
        cv2.imshow('window', image2)  # Display the image
        cv2.waitKey(3000)

if __name__ == '__main__':
    main()
