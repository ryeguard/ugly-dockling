print("Starting program")

import numpy as np
import cv2
import cv2.aruco as aruco
from scipy.spatial import distance as dist

def loadCameraParams(cam_name):
    if cam_name is 'runcam_nano3':
        camera_matrix = np.array([[269.11459175467655, 0.0, 318.8896785174727], [0.0, 262.62554966204, 248.54894259248005], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.1913802179616581, 0.04076781232772304, -0.0014647104190866982, 0.00047321030718756505, -0.0043907605166862065]])
        calibration_error = 0.4467944666063116
    elif cam_name is 'runcam_nanolth':
        camera_matrix = np.array([[333.1833852111547, 0.0, 327.7723204462851], [0.0, 337.3376244908818, 229.96013817983925], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.37915663345130246, 0.18478180306843126, -0.00021990379249122642, -0.0014864903771132248, -0.05061040147030076]])
        calibration_error = 0.5077483684005625
    else:
        print('Camera not found. Returning shit values.')
        camera_matrix = np.array([[1000.0, 0.0, 655], [0.0, 1000.0, 380], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.2, -1.3, -4285.0, -2510.0, 2.3]])
        calibration_error = 100

    return camera_matrix, camera_dist, calibration_error

def drawCrosshair(frame):
    cv2.drawMarker(frame, (int(resolution[0]/2),int(resolution[1]/2)),(0,0,0),markerType=cv2.MARKER_CROSS, thickness=2)

print('OpenCV version: ',cv2.__version__)

font = cv2.FONT_HERSHEY_PLAIN

# Create VideoCapture object
cap = cv2.VideoCapture(2)

camera_matrix, camera_dist, _ = loadCameraParams('runcam_nanolth')
print(camera_dist)

resolution = (640,480)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()
marker_size = 0.132 #0.0265 #0.132
axis_size = marker_size/2.0

#--- Correct displayed image using camera calibration
new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix,
    camera_dist,
    resolution,
    0)
mapx, mapy = cv2.initUndistortRectifyMap(
    camera_matrix,
    camera_dist,
    None,
    new_camera_matrix,
    resolution,
    5)
 
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if mapx is not None and mapy is not None:
        frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)   

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(7,7),0)

    corners, ids, rejected = aruco.detectMarkers(image=gray,dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_dist)
    
    if ids is not None:
        
        aruco.drawDetectedMarkers(frame,corners)
        firstMarkerCorners = corners[0]
        x1,y1 = firstMarkerCorners[0][0].ravel()
        x2,y2 = firstMarkerCorners[0][1].ravel()
        
        cv2.line(frame, (x1,y1), (x2,y2),(255,0,0),thickness=2,lineType=0)
        p1 = np.array(x1,y1)
        p2 = np.array(x2,y2)

        length = int(dist.euclidean((x1,y1),(x2,y2)))
        
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_dist)
        
        for i in range(len(rvecs)):
            aruco.drawAxis(frame, camera_matrix, camera_dist, rvecs[i,0,:], tvecs[i,0,:], axis_size)
        
        cv2.line(frame, (10,10),(10+length,10),(0,255,0),thickness=5,lineType=1)
        str_notdetect = 'Length: '+str(length)+' px'
        cv2.putText(frame, str_notdetect, (10, 40), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        
    
    else: 
        str_notdetect = "No marker detected"
        cv2.putText(frame, str_notdetect, (10, 40), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
    
    # Quit if q is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.imshow('frame', frame)

# When everything done, release the capture and destroy windows
cap.release()
cv2.destroyAllWindows()

print("Ending program")