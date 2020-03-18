# import the necessary packages
import argparse
import cv2
import math

# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []

def click_and_crop(event, x, y, flags, param):

    # grab references to the global variables
    global refPt, cropping
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt.clear()
        refPt.append((x,y))
        cropping = True
    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        refPt.append((x, y))

        # draw a rectangle around the region of interest
        cv2.line(image, refPt[0], refPt[1], (0,0,0), thickness=2)
        cv2.imshow("image", image)

        angle = math.atan2(
            (((refPt[1][1]-480)*-1)-((refPt[0][1]-480)*-1)), 
            (refPt[1][0]-refPt[0][0]))
        print("Angular error: " +str(math.degrees(angle)-90))

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = vars(ap.parse_args())

# load the image, clone it, and setup the mouse callback function
image = cv2.imread(args["image"])
clone = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)

# keep looping until the 'q' key is pressed
while True:
    # display the image and wait for a keypress
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF
    # if the 'c' key is pressed, break from the loop
    if key == ord("q"):
        break

# close all open windows
cv2.destroyAllWindows()
