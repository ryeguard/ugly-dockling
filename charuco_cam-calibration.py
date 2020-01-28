import cv2
from cv2 import aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
img = board.draw((200,200))
cv2.imwrite('charuco.png',img)

cap = cv2.VideoCapture(0)

allCharucoCorners = []
allCharucoIds = []

for i in range(1):

    # Capture and image
    ret,frame = cap.read()

    # Convert to gray scale
    grayframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Detect markers
    res = aruco.detectMarkers(grayframe,dictionary)
    print(res)

cap.release()
cv2.destroyAllWindows()