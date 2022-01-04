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
# Script to capture and show the webcam video, and save a frame if you want
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# ---------------------------------------------------
# Global Variables
# ---------------------------------------------------


def main():
    # ---------------------------------------------------
    # Initialization
    # ---------------------------------------------------

    # initial setup
    capture = cv2.VideoCapture(0)  # Selecting the camera 0
    window_name = 'Webcam'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Load the cascade
    face_cascade = cv2.CascadeClassifier('/home/rodrigo/catkin_ws/src/PSR/opencv/data/haarcascades/haarcascade_frontalface_default.xml')

    print('Start capturing the webcam video.')
    print('press s to save the current frame.')
    print('press q to exit.')
    saved_frame_count = 0
    while True:
        _, frame = capture.read()  # get an image from the camera (a frame)
        overlay = frame.copy()  # create a copy of the frame for transparency process

        # If we want to operate something in the frames captured, just make the operation and show after. Example:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(gray, (3, 3), 0)

        # Canny Edge Detection
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)  # Canny Edge Detection

        # Detect faces in gray image
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)

        mask = edges.astype(bool)  # Convert the edges from uint8 to boolean

        # Draw filled rectangle around the faces and change transparency
        for (x, y, w, h) in faces:

            # Where the rectangle should be, put False on mask
            mask[y:y+h, x:x+w] = False

            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), -1)
            alpha = 0.15  # Transparency factor.
            # Following line overlays transparent rectangle over the image
            frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        # Change the pixels where we have edges to red.
        frame[mask] = (0, 0, 255)  # Where the mask is true, change the pixels to red

        # Show image
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1)

        # Save a frame pressing a key
        if key == ord('s'):
            filename = 'webcam_capture_with_face_detection' + str(saved_frame_count) + '.jpg'
            cv2.imwrite(filename, frame)
            print(filename + 'saved.')
            saved_frame_count += 1

        # key to break the cycle
        if key == ord('q'):
            print('You pressed q, exiting the program')
            break

    # Release everything if job is finished
    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
