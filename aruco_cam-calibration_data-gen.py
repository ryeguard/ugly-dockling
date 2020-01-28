'''This script is for generating data
1. Provide desired path to store images.
2. Press 'c' to capture image and display it.
3. Press any button to continue.
4. Press 'q' to quit.
'''

import os 
import cv2
from pathlib import Path

camera = cv2.VideoCapture(0)
ret, img = camera.read()

root = Path(__file__).parent.absolute()
path = root.joinpath("webcam")
img_path = str(root.joinpath("webcam/"))

count = 0
while True:
    name = img_path + str(count)+'.jpg'
    ret, img = camera.read()
    cv2.imshow("img", img)


    if cv2.waitKey(20) & 0xFF == ord('c'):
        cv2.imwrite(name, img)
        cv2.imshow("img", img)
        count += 1
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
