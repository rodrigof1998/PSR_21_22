#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import cv2
import numpy as np
from functools import partial

# Global variables
# window_name = 'Binarizing image through a track bar'
def onMouse(evento,x,y,flags,param):
    """
    Function called by cv2.setMouseCallback to print the coordinates of where you clicked in the image
    :param action: to click a button in the mouse, specifically the left button down
    :param x: the x coordinate of the image where you clicked
    :param y: the y coordinate of the image where you clicked
    :param flags:
    :param param:
    """
    if evento==cv2.EVENT_LBUTTONDOWN:
        print('As coodenadas do rato são: '+str(x)+', '+str(y))


def onTrackbar(threshold, image_gray, window_name):
    """
    Binarize image using a trackbar callback function
    :param threshold: the threshold of the binarization. data type: int
    """

    # Binarize image and show using the given threshold at the time
    _, image_thresholded = cv2.threshold(image_gray, threshold, 255, cv2.THRESH_BINARY)
    cv2.imshow(window_name, image_thresholded)  # Display the image again


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filename', required=True, help="Input full path image filename")
    args = vars(ap.parse_args())

    # Load image with given filename and show
    image_filename = args['filename']
    image = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.imshow('Original', image)  # Display the image

    # Convert colored image to grayscale and show
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow('Grayscale', cv2.WINDOW_NORMAL)
    cv2.imshow('Grayscale', image_gray)  # Display the image

    # Create window first
    window_name = 'Binarizing image through a track bar'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Create a partial function
    onTrackbar_partial = partial(onTrackbar, image_gray=image_gray, window_name=window_name)

    # Create a trackbar to control the threshold of the binarization
    trackbar_name = 'Binarize threshold'
    cv2.createTrackbar(trackbar_name, window_name, 0, 255, onTrackbar_partial)

    # Initialize binarized image with threshold equal to 0
    onTrackbar_partial(0)

    cv2.setMouseCallback(window_name, onMouse)

    cv2.waitKey(0)  # wait a key


if __name__ == "__main__":
    main()
