#!/usr/bin/python3
import string

import cv2
import argparse
import time

def main():

    # Process arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-path', '--file_location', type=str, required=True,
                    help='File directory1')
    args = vars(ap.parse_args())

    # Load image
    image_filename = args['file_location']
    image_original = cv2.imread(image_filename, cv2.IMREAD_COLOR) # Load an image

    # Process image
    image_gray = cv2.cvtColor(image_original, cv2.COLOR_BGR2GRAY)
    _, image_processed = cv2.threshold(image_gray, 128,255 , cv2.THRESH_BINARY)

    #2b
    image_thresholded = image_gray > 128

    print(type(image_original)) #Print data type of iamge origial
    print(image_original.shape) #Print shape of image original
    print(image_original.dtype) #Print data type of the numeber od numpy

    print(type(image_gray))
    print(image_gray.shape)
    print(image_gray.dtype)

    print(type(image_processed))
    print(image_processed.shape)
    print(image_processed.dtype)

    # Visualization
    cv2.imshow('Original', image_original)  # Display the image
    cv2.imshow('Gray', image_gray)  # Display the image
    cv2.imshow('processed', image_processed)  # Display the image
    cv2.imshow('precessed_2b', image_thresholded)  # Display the image
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
