#!/usr/bin/python3
import cv2

def main():
    # initial setup
    # initial setup
    # Load the cascade
    face_cascade = cv2.CascadeClassifier('/home/rodrigo/Documents/PSR/opencv/data/haarcascades/haarcascade_frontalface_default.xml')

    capture = cv2.VideoCapture(0)  # Selecting the camera 1

    while True:
        # Read the frame
        _, img = capture.read()
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect the faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        # Draw the rectangle around each face
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        # Display
        cv2.imshow('img', img)
        # Stop if escape key is pressed
        k = cv2.waitKey(10) & 0xff
        if k==ord('q'):
            break
    # Release the VideoCapture object
    capture.release()

if __name__ == '__main__':
    main()
