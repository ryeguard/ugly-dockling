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
    elif cam_name is 'runcam_nano3_matlab':
        camera_matrix = np.array([[272.4886845332201, 0.0, 320.4644480817673], [0.0, 267.8810513665159, 247.7275639090179], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.196829273044116, 0.041379816915944, 0.0, 0.0, -0.004194588440859]])
        calibration_error = 0.1606
    else:
        print('Camera not found. Returning shit values.')
        camera_matrix = np.array([[1000.0, 0.0, 655], [0.0, 1000.0, 380], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.2, -1.3, -4285.0, -2510.0, 2.3]])
        calibration_error = 100

    return camera_matrix, camera_dist, calibration_error

def drawCrosshair(frame):
    cv2.drawMarker(frame, (int(resolution[0]/2),int(resolution[1]/2)),(0,0,0),markerType=cv2.MARKER_CROSS, thickness=2)

def save_frame(name, frame):
    """Saves a frame with name and date/time stamp."""
    #now = datetime.datetime.now()
    #date = now.strftime("%Y-%m-%d_%H%M%S")
    cv2.imwrite('calibFrame_board_'+name+'_.jpg',frame)


print('OpenCV version: ',cv2.__version__)

font = cv2.FONT_HERSHEY_PLAIN
alpha = 0.0
resolution = (640,480)

# Create VideoCapture object
cap = cv2.VideoCapture(2)

camera_matrix, camera_dist, _ = loadCameraParams('runcam_nano3')
new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_dist, resolution, alpha=alpha)
mapx1, mapy1 = cv2.initUndistortRectifyMap(camera_matrix, camera_dist, None, new_camera_matrix, resolution, 5)

camera_matrix, camera_dist, _ = loadCameraParams('runcam_nano3_matlab')
new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_dist, resolution, alpha=alpha)
mapx2, mapy2 = cv2.initUndistortRectifyMap(camera_matrix, camera_dist, None, new_camera_matrix, resolution, 5)

k = np.array([[274.47184355, 0.0, 318.80644416], [0.0, 269.65234395, 245.49289061], [0.0, 0.0, 1.0]])
d = np.array([0.13514225, -0.0147262, -0.04347712, 0.02299276])
P =	cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, resolution, np.eye(3))
mapx3, mapy3 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), P, resolution, cv2.CV_32FC1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if mapx1 is not None and mapy1 is not None:
        radtan_frame = cv2.remap(frame, mapx1, mapy1, cv2.INTER_LINEAR) 
        radtan_matlab_frame = cv2.remap(frame, mapx2, mapy2, cv2.INTER_LINEAR) 
        equidist_frame = cv2.remap(frame, mapx3, mapy3, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)  

    
    # Quit if q is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #save_frame('dist', frame)
        #save_frame('radtan', radtan_frame)
        #save_frame('radtan_matlab', radtan_matlab_frame)
        #save_frame('equidist', equidist_frame)
        break
        
    cv2.imshow('dist', frame)
    cv2.imshow('radtan', radtan_frame)
    cv2.imshow('radtan_matlab', radtan_matlab_frame)
    cv2.imshow('equidist', equidist_frame)

# When everything done, release the capture and destroy windows

cap.release()
cv2.destroyAllWindows()

print("Ending program")