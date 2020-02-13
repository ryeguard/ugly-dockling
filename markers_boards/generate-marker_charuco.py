import cv2
from cv2 import aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

nx = int(input('Define board size, # of squares in x [int]: '))
ny = int(input('Define board size, # of squares in y [int]: '))
wc = float(input('Define marker size, chess square size in m [float] (e.g. 0.025): '))
wa = float(input('Define marker size, aruco square size in m [float] (e.g. 0.0175): '))
board = aruco.CharucoBoard_create(nx,ny,wc,wa,dictionary)

img = board.draw((9*500,6*500))
cv2.imwrite('charuco_board.png',img)