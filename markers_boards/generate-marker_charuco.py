import cv2
from cv2 import aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard_create(9,6,.025,.0175,dictionary)
img = board.draw((9*500,6*500))
cv2.imwrite('charuco_board.png',img)