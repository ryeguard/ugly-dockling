import cv2
from cv2 import aruco

# Define your marker
print('Define your marker')
id = int(input('Define marker ID [int]: '))
size = int(input('Define marker size in pizels [int]: '))
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Generate, save and show
img = aruco.drawMarker(aruco_dict,id,size)
cv2.imwrite('aruco'+str(id)+'_'+str(size)+'.png',img)
print('Saved in current folder. Showing marker.')
cv2.imshow('marker',img)

cv2.waitKey(0)
cv2.destroyAllWindows()