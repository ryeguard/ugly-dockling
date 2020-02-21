import numpy as np
import sys, time, math
import threading

# OpenCV & Aruco
import cv2 as cv2
import cv2.aruco as aruco

# Crazyflie
import cflib.crtp # cf real-time protocol
from cflib.crazyflie import Crazyflie 
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import ugly_const

class controlMessage:
    def __init__(self):
        self.errorx = 0.0
        self.errory = 0.0
        self.errorz = 0.0
        self.erroryaw = 0.0


#-- 180 deg rotation matrix around the x axis
R_flipx  = np.zeros((3,3), dtype=np.float32)
R_flipx[0,0] = 1.0
R_flipx[1,1] =-1.0
R_flipx[2,2] =-1.0

#-- 90 deg rotation matrix around z axis
R_flipz = np.zeros((3,3), dtype=np.float32)
#
#
#

#-- Translation relating large marker to small (8.935 cm in marker frame y direction)
T_slide = np.zeros((3,1),dtype=np.float32)
T_slide[1] = 0.08935

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

def init_cv():
    cap = cv2.VideoCapture(ugly_const.CAM_NR)
    res = (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, res

def drawHUD(frame, resolution, yaw_camera):
    midpointx = int(resolution[0]/2)
    midpointy = int(resolution[1]/2)

    #-- Crosshair
    cv2.drawMarker(frame, (midpointx,midpointy),(0,0,0), markerType=cv2.MARKER_CROSS, thickness=2)

    #-- Anglometer
    cv2.ellipse(frame, (midpointx,midpointy),(10,10), -90, 0, -math.degrees(yaw_camera), ugly_const.BLACK, thickness=3)

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

def init_cf():
    #--- Scan for cf
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()

    if len(available) == 0:
        print("No cf found, aborting cf code.")
        return None, None
    else: 
        print('Crazyflies found:')
        for i in available:
            print(str(i[0]))
        URI = 'radio://0/80/2M' #str(available[0])
        cf = Crazyflie(rw_cache='./cache')
    
        return cf, URI

class ComputerVisionThread(threading.Thread):
    def __init__(self,ctrl_message):
        """ constructor, setting initial variables """
        super(ComputerVisionThread,self).__init__()
        self._stopevent = threading.Event()
        self.ctrl_message = ctrl_message

    def run(self):
        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters  = aruco.DetectorParameters_create()
        font = cv2.FONT_HERSHEY_PLAIN
        id2find = [0, 1]
        marker_size  = [0.112, 0.0215]

        camera_matrix, camera_distortion, _ = loadCameraParams('webcam')
        
        cap, resolution = init_cv()
        ids_seen = [0, 0]
        id2follow = 0
        while True:
            ret, frame = cap.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
            
            
            
            if ids is not None:

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                
                #-- Calculate which marker has been seen most at late
                if 0 in ids:
                    ids_seen[0] += 1
                else: 
                    ids_seen[0] = 0
                
                if 1 in ids:
                    ids_seen[1] += 1
                else: 
                    ids_seen[1] = 0
                
                id2follow = np.argmax(ids_seen)
                idx_r, idx_c = np.where(ids == id2follow)
                
                #-- Extract the id to follow 
                corners = np.asarray(corners)
                corners = corners[idx_r, :]
               
                #-- Estimate poses of detected markers
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size[id2follow], camera_matrix, camera_distortion)
                
                #-- Unpack the output, get only the first
                rvec, tvec = rvecs[0,0,:], tvecs[0,0,:]     

                # Draw the detected markers axis
                for i in range(len(rvecs)):
                    aruco.drawAxis(frame, camera_matrix, camera_distortion, rvecs[i,0,:], tvecs[i,0,:], marker_size[id2follow]/2)
                
                #-- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                #-- Now get Position and attitude of the camera respect to the marker
                if id2follow == 0:
                    pos_camera = -R_tc*(np.matrix(tvec).T-T_slide)
                else:
                    pos_camera = -R_tc*(np.matrix(tvec).T)

                str_position = "Position error: x=%4.4f  y=%4.4f  z=%4.4f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                cv2.putText(frame, str_position, (0, 20), font, 1, ugly_const.BLACK, 2, cv2.LINE_AA)

                #-- Get the attitude of the camera respect to the frame
                roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flipx*R_tc)
                str_attitude = "Anglular error: roll=%4.4f  pitch=%4.4f  yaw (z)=%4.4f"%(math.degrees(roll_camera),math.degrees(pitch_camera),math.degrees(yaw_camera))
                cv2.putText(frame, str_attitude, (0, 40), font, 1, ugly_const.BLACK, 2, cv2.LINE_AA)
            
                drawHUD(frame,resolution, yaw_camera)
                pos_flip = np.array([[-pos_camera.item(1)], [pos_camera.item(0)]])

                cmd_flip = np.array([[np.cos(yaw_camera), -np.sin(yaw_camera)], [np.sin(yaw_camera), np.cos(yaw_camera)]])
                pos_cmd = cmd_flip.dot(pos_flip) #cmd_flip*pos_flip
                #print('pos_cmd: ')
                #print(pos_cmd)

                self.ctrl_message.errorx = pos_camera[0]
                self.ctrl_message.errory = pos_camera[1]
                self.ctrl_message.errorz = pos_camera[2]
                self.ctrl_message.erroryaw = math.degrees(yaw_camera)

            cv2.imshow('frame', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()

class CrazyflieThread(threading.Thread):
    def __init__(self, ctrl_message):
        super(CrazyflieThread,self).__init__()
        self._stopevent = threading.Event()
        self.ctrl_message = ctrl_message

    def run(self):
        t = threading.current_thread()
        #--- Scan for cf
        cflib.crtp.init_drivers(enable_debug_driver=False)
        print('Scanning interfaces for Crazyflies...')
        available = cflib.crtp.scan_interfaces()

        if len(available) == 0:
            print("No cf found, aborting cf code.")
            cf = None
        else: 
            print('Crazyflies found:')
            for i in available:
                print(str(i[0]))
            URI = 'radio://0/80/2M' #available[0][0]
            cf = Crazyflie(rw_cache='./cache')
        if cf is None:
            print('Not running cf code.')
            return
        
        # while not self._stopevent.isSet():
        #     print(self.ctrl_message.erroryaw)
        #     time.sleep(1)
        # print('Stopping cf')
        
        with SyncCrazyflie(URI,cf=Crazyflie(rw_cache='./cache')) as scf:
            # We take off when the commander is created
            with MotionCommander(scf) as mc:
                mc.up(0.2,0.5)
                while not self._stopevent.isSet():
                    time.sleep(0.03)
                    cmd_yaw = self.ctrl_message.erroryaw*ugly_const.Kyaw
                    if cmd_yaw > 0.5:
                        mc._set_vel_setpoint(0.0,0.0,0.0,-cmd_yaw)
                    else:
                        cmd_x = self.ctrl_message.errorx*ugly_const.Kx
                        cmd_y = self.ctrl_message.errory*ugly_const.Ky
                        mc._set_vel_setpoint(-cmd_y,cmd_x,0.0,-cmd_yaw)
                    
                    
                    print(cmd_x)
                
                # We land when the commander goes out of scope
                
                print('Landing...')
                
        print('Stopping cf_thread')
        
            

if __name__ == '__main__':
    print('Python version: '+sys.version)
    print('OpenCV version: '+cv2.__version__)

    ctrl_message = controlMessage()
    
    cv_thread = ComputerVisionThread(ctrl_message)
    cf_thread = CrazyflieThread(ctrl_message)

    #cv_thread = threading.Thread(target=cv_thread_function, args=(ctrl_message,))
    #cf_thread = threading.Thread(target=cf_thread_function, args=(ctrl_message,))

    cv_thread.start()
    cf_thread.start()

    #-- Stopping threads
    cv_thread.join()
    print('cv_thread stopped.')
    cf_thread._stopevent.set()
    cf_thread.join()
    print('Both threads stopped.')

        




