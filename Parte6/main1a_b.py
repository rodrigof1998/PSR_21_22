#!/usr/bin/python3
import string

import cv2
import argparse
import time

def onMouse(action, x, y, flags, param):
    """
    Function called by cv2.setMouseCallback to print the coordinates of where you clicked in the image
    :param action: to click a button in the mouse, specifically the left button down
    :param x: the x coordinate of the image where you clicked
    :param y: the y coordinate of the image where you clicked
    :param flags: Not used, but needed to the function works
    :param param: Not used, but needed to the function works
    """
    #Calcula o centro em que o shape[0] tem a altura e o shape[1] tem a largura
    center=(int(param.shape[1]/2),int(param.shape[0]/2))
    texto='PSR'
    font = cv2.FONT_HERSHEY_SIMPLEX

    if action == cv2.EVENT_LBUTTONDOWN:
        param=cv2.circle(param,center,100,(255,0,0),2)
        param=cv2.putText(param,texto,center,font,1,(0,0,255),2)
        cv2.imshow('Original', param)  # Display the image

def main():

    # Process arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--file', type=str, required=True,
                    help='File directory1')
    args = vars(ap.parse_args())

    # Load image
    # Load image with given filename and show
    image_filename = args['file']
    image_rgb = cv2.imread(image_filename, cv2.IMREAD_COLOR)
    window_name='Original'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, image_rgb)  # Display the image

    cv2.setMouseCallback(window_name, onMouse,image_rgb)



    cv2.waitKey(0)

if __name__ == '__main__':
    main()
