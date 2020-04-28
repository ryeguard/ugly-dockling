import numpy as np
import sys, time, math, timeit
import threading
from threading import Barrier

# OpenCV & Aruco
import cv2 as cv2
import cv2.aruco as aruco

# Crazyflie
import cflib.crtp # cf real-time protocol
from cflib.crazyflie import Crazyflie 
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Project specific imports
import uglyConst
from batteryLog import UglyLogger

# Logging
import datetime

class controlMessage:
    def __init__(self):
        self.errorx = 0.0
        self.errory = 0.0
        self.errorz = 0.0
        self.errorpixx = 0.0
        self.errorpixy = 0.0
        self.erroryaw = 0.0
        self.cmd_zvel = 0.0

class stateMessage:
    def __init__(self):
        self.isMarkerDetected = False
        self.cv_mode = uglyConst.CVMODE_PIX
        self.position = np.array([0.0, 0.0, 0.0])
        self.attidtude = np.array([0.0, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0

#-- 180 deg rotation matrix around the x axis
R_flipx  = np.zeros((3,3), dtype=np.float32)
R_flipx[0,0] = 1.0
R_flipx[1,1] =-1.0
R_flipx[2,2] =-1.0

#-- 90 deg rotation matrix around z axis
R_flipz = np.zeros((3,3), dtype=np.float32)
#

#-- Translation relating large marker to small (8.935 cm in marker frame y direction)
T_slide = np.zeros((3,1),dtype=np.float32)
T_slide[1] = uglyConst.MARKER_OFFSET

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def isRotationArray(R):
    Rt = R.transpose()
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationArray(np.array(R)))

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

def rotationArrayToEulerAngles(R):
    assert (isRotationArray(R))

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


class ComputerVisionThread(threading.Thread):
    def __init__(self,ctrl_message, state_message, message_lock, barrier):
        """ constructor, setting initial variables """
        super(ComputerVisionThread,self).__init__()
        self.ctrl = ctrl_message
        self.state = state_message
        self.lock = message_lock
        self.b = barrier

    def init_cv(self):
        cap = cv2.VideoCapture(uglyConst.CAM_NR)
        res = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        return cap, res

    def draw_str(self, dst, x, y, s):
        cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
        cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), lineType = cv2.LINE_AA)

    def draw_HUD(self, frame, resolution, yaw_camera):
        midpointx = int(resolution[0]/2)
        midpointy = int(resolution[1]/2)

        #-- Error crosshair
        cv2.drawMarker(frame, (midpointx-self.ctrl.errory*200,midpointy-self.ctrl.errorx*200),uglyConst.RED, markerType=cv2.MARKER_CROSS, thickness=2)
        
        #-- Anglometer
        cv2.ellipse(frame, (midpointx,midpointy), (10,10), -90, 0, -math.degrees(yaw_camera), uglyConst.BLACK, thickness=3)

    def loadCameraParams(self, cam_name):
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

    def save_frame(self, frame, name):
        now = datetime.datetime.now()
        date = now.strftime("%Y-%m-%d_%H%M")
        cv2.imwrite('figures/endFrame_'+name+'_'+date+'.jpg',frame)

    def undist_frame(self, frame, camera_matrix, camera_dist, resolution):
        new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_dist, (int(resolution[0]), int(resolution[1])), 0)
        mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, camera_dist, None, new_camera_matrix, (int(resolution[0]), int(resolution[1])), 5)
        if mapx is not None and mapy is not None:
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            return True, frame
        else: 
            return False, frame

    def run(self):
        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters  = aruco.DetectorParameters_create()
        parameters.errorCorrectionRate = 1
        font = cv2.FONT_HERSHEY_PLAIN
        id2find = [uglyConst.MARKERID_BIG, uglyConst.MARKERID_SMALL] 
        marker_size  = [uglyConst.MARKERSIZE_BIG, uglyConst.MARKERSIZE_SMALL]

        camera_matrix, camera_distortion, _ = self.loadCameraParams('runcam_nano3')
        
        cap, resolution = self.init_cv()
        ids_seen = [0, 0]
        id2follow = 0
        sumTime = 0.0
        frameCount = 0
        #logFile = open("cv_log.txt","w")
        firstPass = True

        self.b.wait() # barrier to wait for CF thread
        startTime = time.time()*1000
        
        while True:
            t0 = time.time() # start time for fps calculation
            ret, frame = cap.read()

            #-- Convert to grayscale and undistort image           
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #ret, frame = self.undist_frame(frame, camera_matrix, camera_distortion, resolution)

            #-- Detect markers
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)

            #-- Middle crosshair
            cv2.drawMarker(frame, (int(resolution[0]/2),int(resolution[1]/2)),(0,0,0), markerType=cv2.MARKER_CROSS, thickness=2)

            #-- Update frame
            cv2.imshow('frame', frame)

            #-- User input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                #-- End thread
                self.save_frame(frame, 'dist')
                ret, frame = self.undist_frame(frame, camera_matrix, camera_distortion, resolution)
                if ret:
                    self.save_frame(frame, 'rect')
                cap.release()
                cv2.destroyAllWindows()
                #logFile.close()
                print("Pressed 'Q' to exit CV thread.")
 
class CrazyflieThread(threading.Thread):
    def __init__(self, ctrl_message, state_message, message_lock, barrier):
        threading.Thread.__init__(self)
        self.ctrl = ctrl_message
        self.state = state_message
        self.lock = message_lock
        self.cf = None
        self.mc = None
        self.scf = None
        self.runSM = True
        self.cmd_height_old = uglyConst.TAKEOFF_HEIGHT
        self.landingController = uglyConst.LANDMODE_NONE # see uglyConst for controller choice
        self.b = barrier

    def raise_exception(self):
        self.runSM = False

    #-- FSM condition funcitons
    def isCloseXYP(self, r):
        x = self.ctrl.errorx
        y = self.ctrl.errory
        if (np.sqrt(x*x+y*y) > r) or (np.abs(self.ctrl.erroryaw) > uglyConst.FAR_ANGL):
            return False
        else:
            return True

    def isCloseCone(self):
        x = self.ctrl.errorx
        y = self.ctrl.errory
        r = (self.mc._thread.get_height()-uglyConst.DIST_IGE)*uglyConst.FAR_CONE
        if (np.sqrt(x*x+y*y) > r): # or (np.abs(self.ctrl.erroryaw) > uglyConst.FAR_ANGL):
            return False
        else:
            return True

    def isClosePix(self):
        x = self.ctrl.errorpixx
        y = self.ctrl.errorpixy
        if (np.sqrt(x*x+y*y) > 50):
            return False
        else:
            return True

    def limOutputVel(self, vx, vy, vz):
        return min(vx, uglyConst.MAX_XVEL), min(vy, uglyConst.MAX_YVEL), min(vz, uglyConst.MAX_ZVEL)
    
    #-- FSM state functions
    def stateInit(self):
        #--- Scan for cf
        cflib.crtp.init_drivers(enable_debug_driver=False)
        print('Scanning interfaces for Crazyflies...')
        available = cflib.crtp.scan_interfaces()

        if len(available) == 0:
            print("No cf found, aborting cf code.")
            self.cf = None
        else: 
            print('Crazyflies found:')
            for i in available:
                print(str(i[0]))
            self.URI = 'radio://0/80/2M' #available[0][0]
            self.cf = Crazyflie(rw_cache='./cache')
        
        if self.cf is None:
            self.b.wait()
            print('Not running cf code.')
            return None
        
        self.scf = SyncCrazyflie(self.URI,cf=Crazyflie(rw_cache='./cache'))
        self.scf.open_link()
        self.mc = MotionCommander(self.scf)
        self.file = open("bat_log_camoff7.txt","a")

        #-- Barrier to wait for CV thread
        self.b.wait()
        self.lgr = UglyLogger(self.URI, self.scf, self.file)

        self.enterTakingOff()
        return self.stateTakingOff

    def enterTakingOff(self):
        print("enterTakingOff")
        self.mc.take_off(uglyConst.TAKEOFF_HEIGHT,uglyConst.TAKEOFF_ZVEL)

    def stateTakingOff(self):
        print("stateTakingOff")
        
        if  self.mc._thread.get_height() >= uglyConst.TAKEOFF_HEIGHT:
            return self.stateSeeking
        else:
            time.sleep(0.05)
            return self.stateTakingOff

    def stateSeeking(self):
        print("stateSeeking")
        self.mc._set_vel_setpoint(0.0, 0.0, 0.0, 0.0)
        
        time.sleep(1)
        batlev = self.lgr.get_batlev()
        print(batlev)
        if self.lgr.get_batlev() < 3.0:
            return self.stateLanding
        return self.stateSeeking

    def stateLanding(self):
        print("stateLandning")
        self.mc.stop()
        self.mc.land()
        return self.stateLanded

    def stateLanded(self):
        print("stateLanded")
        return None

    def run(self):
        try: 
            state = self.stateInit    # initial state
            while state and self.runSM: 
                state = state()

        finally:
            print('Stopping cf_thread')
            if self.cf is not None:
                if self.mc._is_flying:
                    self.mc.stop()
                    print('Finally landing')
                    self.mc.land()
                self.scf.close_link()
                self.file.close()
                  
if __name__ == '__main__':
    #print('Python version: '+sys.version)
    #print('OpenCV version: '+cv2.__version__)

    state_message = stateMessage()
    ctrl_message = controlMessage()
    message_lock = threading.Lock()

    b = Barrier(2)
    
    cv_thread = ComputerVisionThread(ctrl_message, state_message, message_lock, b)
    cf_thread = CrazyflieThread(ctrl_message, state_message, message_lock, b)

    #-- Starting threads
    cv_thread.start()
    cf_thread.start()

    #-- Stopping threads
    cv_thread.join()
    print('cv_thread stopped.')
    cf_thread.raise_exception()
    cf_thread.join()
    print('cf_thread stopped.')

        