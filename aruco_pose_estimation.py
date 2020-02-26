"""
This demo calculates multiple things for different scenarios.

Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f


"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import cflib as cf

print("OpenCV version: ",cv2.__version__)
cam_id = int(input('Input camera ID: '))

#--- Define Tag
id_to_find  = 0
marker_size  = 13.23 #- [cm]

#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def drawCrosshair(frame):
    cv2.drawMarker(frame, (int(resolution[0]/2),int(resolution[1]/2)),(0,0,0),markerType=cv2.MARKER_CROSS, thickness=2)

#--- Get the camera calibration path

def loadCameraParams(cam_name):
    if cam_name is 'runcam_nano3':
        camera_matrix = np.array([[269.11459175467655, 0.0, 318.8896785174727], [0.0, 262.62554966204, 248.54894259248005], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.1913802179616581, 0.04076781232772304, -0.0014647104190866982, 0.00047321030718756505, -0.0043907605166862065]])
        calibration_error = 0.4467944666063116
    elif cam_name is 'runcam_nanolth':
        camera_matrix = np.array([[333.1833852111547, 0.0, 327.7723204462851], [0.0, 337.3376244908818, 229.96013817983925], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.37915663345130246, 0.18478180306843126, -0.00021990379249122642, -0.0014864903771132248, -0.05061040147030076]])
        calibration_error = 0.5077483684005625
    elif cam_name is 'webcam':
        camera_matrix = np.array([[1000.0, 0.0, 655], [0.0, 1000.0, 380], [0.0, 0.0, 1.0]])
        camera_dist = np.array([[-0.2, -1.3, -.0042, -.0025, 2.3]])
        calibration_error = 100
    else:
        print('Camera not found. Returning shit values.')
        camera_matrix = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        camera_dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        calibration_error = 100

    return camera_matrix, camera_dist, calibration_error

camera_matrix, camera_distortion, _   = loadCameraParams('runcam_nano3')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters  = aruco.DetectorParameters_create()


#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(cam_id)
resolution = (640, 480)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

while True:

    #-- Read the camera frame
    ret, frame = cap.read()

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    
    if ids is not None: # and ids[0] == id_to_find:
        
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        
        #-- Unpack the output, get only the first
        rvec, tvec = rvecs[0,0,:], tvecs[0,0,:]

        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)

        drawCrosshair(frame)
        
        # Draw the detected markers axis
        for i in range(len(rvecs)):
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvecs[i,0,:], tvecs[i,0,:], 6.5)

        #-- Print the tag position in camera frame
        #str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        #cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera frame
        #str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                            #math.degrees(yaw_marker))
        #cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        #-- Now get Position and attitude of the camera respect to the marker
        pos_camera = -R_tc*np.matrix(tvec).T
        str_position = "Position error: x=%4.4f  y=%4.4f  z=%4.4f"%(pos_camera[0], pos_camera[1], pos_camera[2])
        cv2.putText(frame, str_position, (0, 20), font, 1, (0, 0, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_attitude = "Anglular error: roll=%4.4f  pitch=%4.4f  yaw (z)=%4.4f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
        cv2.putText(frame, str_attitude, (0, 40), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
        
        


    #--- Display the frame
    cv2.imshow('frame', frame)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break































