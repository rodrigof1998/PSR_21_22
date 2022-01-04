#!/usr/bin/python3
import cv2

def main():
    # initial setup
    # initial setup

    capture = cv2.VideoCapture(0)  # Selecting the camera 0
    window_name = 'Webcam'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:

        _, frame = capture.read()  # get an image from the camera (a frame)
        # Show image
        cv2.imshow(window_name, frame)




        cv2.waitKey(10)

    # add code to show acquired image
    # add code to wait for a key press

if __name__ == '__main__':
    main()
