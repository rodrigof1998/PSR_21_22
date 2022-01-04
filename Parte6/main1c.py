#!/usr/bin/python3
import string

import cv2
import argparse
import time

import numpy as np

desenhando=False
cor=(0,0,255)

def onMouse(action, x, y, flags, param):
    """
    Function called by cv2.setMouseCallback to print the coordinates of where you clicked in the image
    :param action: to click a button in the mouse, specifically the left button down
    :param x: the x coordinate of the image where you clicked
    :param y: the y coordinate of the image where you clicked
    :param flags: Not used, but needed to the function works
    :param param: Not used, but needed to the function works
    """

    global cor
    global desenhando

    if action == cv2.EVENT_LBUTTONDOWN:
        desenhando=True
        param[y,x]=cor
    elif action== cv2.EVENT_MOUSEMOVE:
        if desenhando:
            param[y,x]=cor
    elif action== cv2.EVENT_LBUTTONUP:
        desenhando=False

def main():


    # Load image
    # Load image with given filename and show

    paint = 255 * np.ones((600, 400, 3), np.uint8)

    window_name = 'Paint power'

    global cor
    print('Click in the image to paint the pixel or click and drag to paint continuously.'
          '\nPress r to change to red color.'
          '\nPress g to change to green color.'
          '\nPress b to change to blue color.'
          '\nInitializing with red color as default.')

    while True:
        #Mostra a imagem todos os ciclos
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, paint)  # Display the image

        key=cv2.waitKey(10)

        if key == ord('r'):
            cor = (0, 0, 255)
            print('You changed the color to red')
        elif key == ord('g'):
            cor = (0, 255, 0)
            print('You changed the color to green')
        elif key == ord('b'):
            cor = (255, 0, 0)
            print('You changed the color to blue')
        elif key == ord('q'):
            print('Letter q (quit) pressed, exiting the program')
            break

        cv2.setMouseCallback(window_name, onMouse,paint)

if __name__ == '__main__':
    main()
