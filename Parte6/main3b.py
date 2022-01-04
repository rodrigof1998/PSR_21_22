#!/usr/bin/python3
import cv2

def main():
    # initial setup
    # initial setup
    # Load the cascade
    face_cascade = cv2.CascadeClassifier('/home/rodrigo/catkin_ws/src/PSR/opencv/data/haarcascades/haarcascade_frontalface_default.xml')

    capture = cv2.VideoCapture(0)  # Selecting the camera 1

    while True:
        # Read the frame
        _, img = capture.read()
        overlay = img.copy()  # create a copy of the frame for transparency process

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(gray, (3, 3), 0)

        # Canny Edge Detection
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)  # Canny Edge Detection

        mask = edges.astype(bool)  # Convert the edges from uint8 to boolean

        # Detect the faces
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)

        # Draw the rectangle around each face
        for (x, y, w, h) in faces:
            alpha = 0.15  # Transparency factor.
            # Following line overlays transparent rectangle over the image
            img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

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
