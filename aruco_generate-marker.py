import cv2
from cv2 import aruco

# Define your marker
id = 0
size = 500 # pixels
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Generate it
img = aruco.drawMarker(aruco_dict,id,size)
cv2.imwrite('markers/aruco'+str(id)+'.png',img)