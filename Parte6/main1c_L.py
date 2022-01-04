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
#
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# ---------------------------------------------------
# Global Variables
# ---------------------------------------------------
color = (0, 0, 255)
drawing = False


# function which will be called on mouse input
def onMouse(action, x, y, flags, param):
    """
    Function called by cv2.setMouseCallback to return the coordinates of where you clicked in the image
    :param action: to click a button in the mouse, specifically the left button down
    :param x: the x coordinate of the image where you clicked
    :param y: the y coordinate of the image where you clicked
    :param flags: Not used, but needed to the function works
    :param param: The image used to draw in
    """
    global drawing
    if action == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        param[y, x] = color
        print('You started to paint in the pixel x = ' + str(x) + ' , y = ' + str(y))

    elif action == cv2.EVENT_MOUSEMOVE:
        if drawing:
            param[y, x] = color

    elif action == cv2.EVENT_LBUTTONUP:
        drawing = False
        print('and finished in the pixel x = ' + str(x) + ' , y = ' + str(y))


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    blank_image = 255 * np.ones((600, 400, 3), np.uint8)
    window_name = 'Image to draw in'
    global color
    print('Click in the image to paint the pixel or click and drag to paint continuously.'
          '\nPress r to change to red color.'
          '\nPress g to change to green color.'
          '\nPress b to change to blue color.'
          '\nInitializing with red color as default.')

    while True:
        # Show image at every cycle
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, blank_image)  # Display the image

        key = cv2.waitKey(10)

        if key == ord('r'):
            color = (0, 0, 255)
            print('You changed the color to red')
        elif key == ord('g'):
            color = (0, 255, 0)
            print('You changed the color to green')
        elif key == ord('b'):
            color = (255, 0, 0)
            print('You changed the color to blue')
        elif key == ord('q'):
            print('Letter q (quit) pressed, exiting the program')
            break

        # Paint where the mouse clicked
        cv2.setMouseCallback(window_name, onMouse, param=blank_image)


if __name__ == "__main__":
    main()
