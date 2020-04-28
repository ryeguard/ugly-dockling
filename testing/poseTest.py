import numpy as np
import sys, time, math, timeit
import threading
from threading import Barrier

# OpenCV & Aruco
import cv2 as cv2
import cv2.aruco as aruco
from scipy.spatial import distance as dist

# Crazyflie
import cflib.crtp # cf real-time protocol
from cflib.crazyflie import Crazyflie 
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Project specific imports
import uglyConst
from controllerLog import UglyLogger

# Logging
import datetime

# Custom data structure (NOTE: currently possible to merge ctrl and state messages)
class controlMessage:
    def __init__(self):
        self.errorx = 0.0
        self.errory = 0.0
        self.errorz = 0.0
        self.errorpixx = 0.0
        self.errorpixy = 0.0
        self.erroryaw = 180.0
        self.cmd_zvel = 0.0
        self.K = 0.0

class stateMessage:
    def __init__(self):
        self.isMarkerDetected = False
        self.cv_mode = uglyConst.CVMODE_POSE
        self.position = np.array([0.0, 0.0, 0.0])
        self.attidtude = np.array([0.0, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0

#-- 180 deg rotation matrix around the x axis
R_flipx  = np.zeros((3,3), dtype=np.float32)
R_flipx[0,0] = 1.0
R_flipx[1,1] =-1.0
R_flipx[2,2] =-1.0

#-- TODO: 90 deg rotation matrix around z axis
R_flipz = np.zeros((3,3), dtype=np.float32)

#-- Translation relating large marker to small (8.935 cm in marker frame y direction)
T_slide = np.zeros((3,1),dtype=np.float32)
T_slide[1] = uglyConst.MARKER_OFFSET

def isRotationMatrix(R):
    """
    Checks if matrix R is a valid rotation matrix. 
    NOTE: Is replaced by isRotationArray b/c np.matrix is legacy code.
    TODO: Remove when replaced by isRotationArray everywhere.
    """
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def isRotationArray(R):
    """
    Checks if matrix R is a valid rotation matrix.
    TODO: Move into CV class.
    """
    Rt = R.transpose()
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    """
    Converts from rotation matrix R representation to Euler angle representation.
    NOTE: Is replaced by rotationArrayToEulerAngles b/c np.matrix is legacy code.
    TODO: Remove when replaced by rotationArrayToEulerAngles everywhere.
    """

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
    """
    Converts from rotation matrix R representation to Euler angle representation.
    TODO: Move into CV class.
    """
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
        
        super(ComputerVisionThread,self).__init__()
        self.ctrl = ctrl_message
        self.state = state_message
        self.lock = message_lock
        self.b = barrier

    def init_cv(self):
        """Open camera capture. Returns cap and resolution."""

        cap = cv2.VideoCapture(uglyConst.CAM_NR)
        res = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        return cap, res

    def draw_str(self, dst, x, y, s):
        """Draws a string s on frame dst at coorinates (x,y)."""
        cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
        cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), lineType = cv2.LINE_AA)

    def draw_HUD(self, frame, resolution, yaw_camera):
        """Draws Heads Up Display on frame frame with resolution resolution."""
        midpointx = int(resolution[0]/2)
        midpointy = int(resolution[1]/2)

        #-- Error crosshair
        cv2.drawMarker(frame, (midpointx-self.ctrl.errory*200,midpointy-self.ctrl.errorx*200),uglyConst.RED, markerType=cv2.MARKER_CROSS, thickness=2)
        
        #-- Anglometer
        cv2.ellipse(frame, (midpointx,midpointy), (10,10), -90, 0, -math.degrees(yaw_camera), uglyConst.BLACK, thickness=3)

    def loadCameraParams(self, cam_name):
        """Returns intrinsic parameters (camera matrix, camera distortions, calibration error) of camera cam_name."""
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
        elif cam_name is 'runcam_nano3_matlab':
            camera_matrix = np.array([[272.4886845332201, 0.0, 320.4644480817673], [0.0, 267.8810513665159, 247.7275639090179], [0.0, 0.0, 1.0]])
            camera_dist = np.array([[-0.196829273044116, 0.041379816915944, -0.004194588440859, 0.0, 0.0]])
            calibration_error = 0.1606
        else:
            print('Camera not found. Returning shit values.')
            camera_matrix = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
            camera_dist = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
            calibration_error = 100

        return camera_matrix, camera_dist, calibration_error

    def save_frame(self, frame, name):
        """Saves a frame with name and date/time stamp."""
        now = datetime.datetime.now()
        date = now.strftime("%Y-%m-%d_%H%M%S")
        cv2.imwrite('poseTest_'+name+'_'+date+'.jpg',frame)

    def undist_frame(self, frame, camera_matrix, camera_dist, resolution):
        """Undistorts frame using camera matrix and distortions. Returns True if successful, False if not."""
        new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_dist, (int(resolution[0]), int(resolution[1])), 0)
        mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, camera_dist, None, new_camera_matrix, (int(resolution[0]), int(resolution[1])), 5)
        if mapx is not None and mapy is not None:
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            return True, frame
        else: 
            return False, frame

    def run(self):
        """Main loop. State machine."""

        #-- Define AruCo
        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters  = aruco.DetectorParameters_create()
        #parameters.errorCorrectionRate = 1

        #-- Define font
        font = cv2.FONT_HERSHEY_PLAIN

        #-- Import marker data from constants
        id2find = [uglyConst.MARKERID_BIG, uglyConst.MARKERID_SMALL] 
        marker_size  = [uglyConst.MARKERSIZE_BIG, uglyConst.MARKERSIZE_SMALL]

        camera_matrix, camera_distortion, _ = self.loadCameraParams('runcam_nano3_matlab')
        cap, resolution = self.init_cv()
        
        #-- Init variable
        ids_seen = [0, 0]
        id2follow = 0
        sumTime = 0.0
        frameCount = 0
        pos_pixx, pos_pixy = 0.0, 0.0
        pos_camera = np.zeros((3,1), dtype=np.float32)
        att_camera = np.zeros((3,1), dtype=np.float32)
        isMarkerDetected = 0
        meanlen = 0.0

        logFile = open("cv_log.txt","w") # open log file
    
        #self.b.wait() # barrier to wait for CF thread, sync
        startTime = time.time()
        
        while True:
            t0 = time.time() # start time for fps calculation
            ret, frame = cap.read()

            #-- Convert to grayscale and undistort image           
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray,(5,5),0)
            #ret, frame = self.undist_frame(frame, camera_matrix, camera_distortion, resolution)

            #-- Detect markers
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
            
            #-- State: Pose estimation
            if ids is not None: 

                isMarkerDetected = 1

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                
                #-- Calculate which marker has been seen most at late
                if id2find[0] in ids:
                    ids_seen[0] += 1
                else: 
                    ids_seen[0] = 0
                
                if id2find[1] in ids:
                    ids_seen[1] += 3 # +2 to choose smaller tag 
                else: 
                    ids_seen[1] = 0
                
                id2follow = 0 # np.argmax(ids_seen) 
                idx_r, idx_c = np.where(ids == id2find[id2follow])
                
                #-- Extract the id to follow
                corners = np.asarray(corners)
                corners = corners[idx_r]
            
                sumlen = 0.0
                meanlen = 0.0
                x1,y1 = 0.0, 0.0

                if len(corners) == 1:
                    for i in range(0,4):
                        x1,y1 = corners[0][0][i].ravel()
                        x2,y2 = corners[0][0][(i+1)%4].ravel()
                        sumlen += dist.euclidean((x1,y1),(x2,y2))

                        xsum = 0.0
                        ysum = 0.0
                        for i in range(4):
                            xsum += corners[0][0][i][0]
                            ysum += corners[0][0][i][1]
                        xavg = xsum/4.0
                        yavg = ysum/4.0
                        pos_pixx = 320-xavg
                        pos_pixy = 240-yavg
                    meanlen = sumlen/4
                    #cv2.line(frame, (int(x1),int(y1)),(int(x1+meanlen),int(y1)),(0,255,0),thickness=5,lineType=1)
               
                #-- Estimate pose of extracted marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size[id2follow], camera_matrix, camera_distortion)
                
                if rvecs is not None:
                    #-- Unpack the output, get only the first
                    rvec, tvec = rvecs[0,0,:], tvecs[0,0,:] 

                    #-- Draw the detected markers axis
                    for i in range(len(rvecs)):
                        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvecs[i,0,:], tvecs[i,0,:], marker_size[id2follow]/2)
                    
                    #-- Obtain the rotation matrix tag->camera
                    R_ct = np.array(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.transpose()

                    #-- Now get Position and attitude of the camera respect to the marker
                    temp = np.array(tvec, ndmin=2).transpose()
                    if id2follow == 0:
                        pos_camera = np.matmul(-R_tc,temp)#-T_slide
                    else:
                        pos_camera = np.matmul(-R_tc,temp)
                    
                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = rotationArrayToEulerAngles(np.matmul(R_flipx,R_tc)) #R_flipx*R_tc)
                    att_camera = [math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)]
                
                    pos_flip = np.array([[-pos_camera.item(1)], [pos_camera.item(0)]])
                    cmd_flip = np.array([[np.cos(yaw_camera), -np.sin(yaw_camera)], [np.sin(yaw_camera), np.cos(yaw_camera)]])
                    pos_cmd = cmd_flip.dot(pos_flip)

                    #-- Draw some information on frame
                    str_position = "Pos: x=%4.4f  y=%4.4f  z=%4.4f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    self.draw_str(frame, 0,40, str_position)
                    str_attitude = "Att: roll=%4.4f  pitch=%4.4f  yaw (z)=%4.4f"%(att_camera[0],att_camera[1],att_camera[2])
                    self.draw_str(frame, 0, 60, str_attitude)

                    #self.draw_HUD(frame, resolution, yaw_camera)
                        
            #-- State: No detected marker
            else:
                isMarkerDetected = 0

            currentTime = time.time()-startTime
            if currentTime > 1.0:
                logFile.write("%f,%f,%f,%f,%f,%f,%f,%d,%f\n" % (currentTime,pos_camera[0],pos_camera[1],pos_camera[2],att_camera[0],att_camera[1],att_camera[2], isMarkerDetected, meanlen))
            
            #-- Calculate and draw FPS on frame
            t1 = time.time() # end time for fps calculation
            sumTime = sumTime+(t1-t0)
            frameCount += 1

            str_fps = "Cur FPS: %4.4f, Avg FPS: %4.4f" %(1.0/(t1-t0), (frameCount/sumTime))
            self.draw_str(frame, 0, 20, str_fps)

            #-- Middle crosshair
            cv2.drawMarker(frame, (int(resolution[0]/2),int(resolution[1]/2)),(0,0,0), markerType=cv2.MARKER_CROSS, thickness=2)

            #-- Update frame
            cv2.imshow('frame', frame)

            #-- User input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or currentTime > 11.0:
                #-- End thread
                self.save_frame(frame, 'dist')
                ret, frame = self.undist_frame(frame, camera_matrix, camera_distortion, resolution)
                if ret:
                    self.save_frame(frame, 'rect')
                cap.release()
                cv2.destroyAllWindows()
                logFile.close()
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
        self.descCounter = 0
        self.startTime = 0.0

    def raise_exception(self):
        self.runSM = False

    #-- FSM condition funcitons
    def isCloseXYP(self, r):
        """Determines if drone is within radius r and aligned."""
        x = self.ctrl.errorx
        y = self.ctrl.errory
        if (np.sqrt(x*x+y*y) > r) or (np.abs(self.ctrl.erroryaw) > uglyConst.FAR_ANGL):
            return False
        else:
            return True

    def isCloseCone(self):
        """Determines if drone within an inverted cone (defined in constants)."""
        x = self.ctrl.errorx
        y = self.ctrl.errory
        r = (self.mc._thread.get_height()-uglyConst.DIST_IGE)*uglyConst.FAR_CONE
        if (np.sqrt(x*x+y*y) > r): # or (np.abs(self.ctrl.erroryaw) > uglyConst.FAR_ANGL):
            return False
        else:
            return True

    def isClosePix(self):
        """Determines if drone is within radius (in pixels, defined in constants)."""
        x = self.ctrl.errorpixx
        y = self.ctrl.errorpixy
        if (np.sqrt(x*x+y*y) > uglyConst.CLOSE_PIX):
            return False
        else:
            return True

    def limOutputVel(self, vx, vy, vz):
        """Limits output of velocity controller."""
        return min(vx, uglyConst.MAX_XVEL), min(vy, uglyConst.MAX_YVEL), min(vz, uglyConst.MAX_ZVEL)
    
    #-- FSM state functions
    def stateInit(self):
        """Initialize CF (scan, open link) and logging framework"""
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
        self.file = open("cf_log.txt","w")

        #-- Barrier to wait for CV thread
        self.b.wait()
        self.lgr = UglyLogger(self.URI, self.scf, self.file)

        self.enterTakingOff()
        return self.stateTakingOff

    def enterTakingOff(self):
        """Entry to state: Take off"""
        print("enterTakingOff")
        self.mc.take_off(uglyConst.TAKEOFF_HEIGHT,uglyConst.TAKEOFF_ZVEL)

    def stateTakingOff(self):
        """State: Taking off"""
        print("stateTakingOff")
        
        if  self.mc._thread.get_height() >= uglyConst.TAKEOFF_HEIGHT:
            return self.stateSeeking
        else:
            time.sleep(0.05)
            return self.stateTakingOff

    def stateSeeking(self):
        """State: Seeking. Slowly ascend until marker is detected. TODO: Implement some kind of search algorithm (circle outward?)"""
        self.mc._set_vel_setpoint(0.0, 0.0, 0.01, 0.0)
        print("stateSeeking")
        
        if self.state.isMarkerDetected:
            return self.stateStepXYStart
        else:
            time.sleep(0.05)
            return self.stateSeeking

    def stateStepXStart(self):
        error = self.ctrl.errorx-1.0
        self.mc._set_vel_setpoint(error*uglyConst.Kx, 0.0, 0.0, 0.0)
        print(error)

        if np.abs(error)<0.001:
            return self.stateStepX
        else:
            time.sleep(0.01)
            return self.stateStepXStart

    def stateStepXYStart(self):
        errorx = self.ctrl.errorx-0.707106/2
        errory = self.ctrl.errory-0.707106/2
        self.mc._set_vel_setpoint(errorx*uglyConst.Kx, errory*uglyConst.Ky, 0.0, 0.0)
        print(errorx, errory)

        if np.abs(errorx)<0.01 and np.abs(errory)<0.01:
            self.descCounter += 1
            if self.descCounter > 10:
                print(self.descCounter)
                self.startTime = time.time() # milliseconds
                return self.stateStepXY
            else: 
                time.sleep(0.01)
                return self.stateStepXYStart
        else:
            time.sleep(0.01)
            return self.stateStepXYStart

    def stateStepX(self):

        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx, 0.0, 0.0, 0.0)
        duration = time.time() - self.startTime
        print(duration)

        if duration > 5.0:
            self.mc.land()
            print("LANDING")
            return stateLanded
        else:
            time.sleep(0.01)
            return self.stateStepX

    def stateStepXY(self):
        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx, self.ctrl.errory*uglyConst.Ky, 0.0, 0.0)
        

        duration = time.time() - self.startTime
        print(duration)
        if duration > 5.0:
            self.mc.land()
            print("LANDING")
            return state.landed
        else:
            time.sleep(0.01)
            return self.stateStepXY

    def stateNearing(self):
        """
        State: Nearing 
        Control in pixel frame. Takes in error in pixels (note: camera coordinates), outputs velocity command in x,y,z. Z vel inversely proportional to radius.
        """
        x = self.ctrl.errorpixx
        y = self.ctrl.errorpixy
        cmdx = y*uglyConst.PIX_Kx
        cmdy = x*uglyConst.PIX_Ky
        r = np.sqrt(x*x+y*y)
        if r > 0.0:
            cmdz = (1/r)*uglyConst.PIX_Kz
        else: 
            cmdz = 1

        cmdx, cmdy, cmdz = self.limOutputVel(cmdx, cmdy, cmdz)
        
        self.mc._set_vel_setpoint(cmdx, cmdy, -cmdz, 0.0)
        print("stateNearing")

        if self.isClosePix() and self.mc._thread.get_height() < uglyConst.NEARING2APPROACH_HEIGHT:
            self.state.cv_mode = uglyConst.CVMODE_POSE
            return self.stateApproachingXY
        else:
            time.sleep(0.05)
            return self.stateNearing

    def stateApproachingXY(self):
        """
        State: Approaching (XY plane)
        Control in the horizontal XY plane and yaw angle.
        """
        #self.mc._set_vel_setpoint(self.ctrl.errorx*(uglyConst.Kx+self.ctrl.K), self.ctrl.errory*(uglyConst.Ky+self.ctrl.K), 0.0, 30.0)
        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx, self.ctrl.errory*uglyConst.Ky, 0.0, -self.ctrl.erroryaw*uglyConst.Kyaw)
        print("stateApproachingXY")
        if not self.isClosePix:
            return self.stateNearing
        if self.isCloseCone() and np.abs(self.ctrl.erroryaw) < uglyConst.FAR_ANGL:
            return self.stateApproachingXYZ
        else:
            time.sleep(0.05)
            return self.stateApproachingXY

    def stateApproachingXYZ(self):
        """
        State: Approaching
        Control in world frame. Takes in error in meters, outputs velocity command in x,y. Constant vel cmd in z.
        """
        
        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx, self.ctrl.errory*uglyConst.Ky, -uglyConst.APPROACH_ZVEL, -self.ctrl.erroryaw*uglyConst.Kyaw)
        print("stateApproachingXYZ")
        
        if not self.isCloseCone:
            return self.stateApproachingXY
        elif self.mc._thread.get_height() < uglyConst.APPROACH2LANDING_HEIGHT:
            if self.landingController == uglyConst.LANDMODE_NONE: 
                return self.stateLanding 
            elif self.landingController == uglyConst.LANDMODE_HOVER:
                return self.stateHoverLand
            elif self.landingController == uglyConst.CTRL_POSD:
                self.enterLandingIGE()
                return self.stateLandingIGE
        else:
            time.sleep(0.05)
            return self.stateApproachingXYZ

    def stateLanding(self):
        """
        State: Landing
        Procedure: Descend to a set height, then stop and land.
        """
        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx*2.0, self.ctrl.errory*uglyConst.Ky*2.0, -uglyConst.LANDING_ZVEL, -self.ctrl.erroryaw*uglyConst.Kyaw)

        print("stateLanding")

        if self.mc._thread.get_height() > uglyConst.APPROACH2LANDING_HEIGHT:
            return self.stateApproaching
        elif self.mc._thread.get_height() < uglyConst.LANDING2LANDED_HEIGHT:
            #self.exitLanding()
            #return self.stateLanded
            return self.stateControlDescend
        else:
            time.sleep(0.01)
            return self.stateLanding

    def stateHoverLand(self):
        """
        State: HoverLanding
        Procedure: Stop at certain height, adjust in XY plane until exactly over tag, then land.
        """
        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx, self.ctrl.errory*uglyConst.Ky, 0.0, -self.ctrl.erroryaw*uglyConst.Kyaw)
        print("stateHoverLand")

        if self.isCloseXYP(uglyConst.LANDING_DIST):
            self.exitLanding()
            return self.stateLanded
        else:
            time.sleep(0.01)
            return self.stateHoverLand

    def enterLandingIGE(self):
        print("enterLandingIGE")
        self.cmd_height_old = self.mc._thread.get_height()
        return self.stateLandingIGE

    def stateLandingIGE(self):
        print("stateLandingIGE")
        cmd_height = ref_height - self.mc._thread.get_height()

        D = cmd_height-self.cmd_height_old

        cmd_zvel = (D-self.ctrl.cmd_zvel) / 2.0 # smoothing, for fixed frame rate!!! :O

        self.mc._set_vel_setpoint(
            self.ctrl.errorx*uglyConst.Kx, 
            self.ctrl.errory*uglyConst.Ky, 
            cmd_zvel*uglyConst.Kz, 
            -self.ctrl.erroryaw*uglyConst.Kyaw)

        self.cmd_height_old = cmd_height

        if self.mc._thread.get_height() > (uglyConst.DIST_IGE + uglyConst.DIST_IGE_HYST):
            return self.stateApproaching
        elif self.mc._thread.get_height() < uglyConst.LANDING_HEIGHT:
            self.exitLanding()
            return self.stateLanded
        else:
            time.sleep(0.05)
            return self.stateLandingIGE

    def stateControlDescend(self):
        """"""
        self.mc._set_vel_setpoint(self.ctrl.errorx*uglyConst.Kx*4.0, self.ctrl.errory*uglyConst.Ky*4.0, -uglyConst.MAX_ZVEL, -self.ctrl.erroryaw*uglyConst.Kyaw*2.0)
        self.descCounter += 1  
        print("stateControlDescend")
        if self.descCounter > 10:
            self.mc.land()
            return self.stateLanded  
        else:   
            time.sleep(0.01)
            return self.stateControlDescend

    def exitLanding(self):
        """
        Exit from state: Landing
        Stop movement (vel cmds=0), then descend.
        """
        self.mc.land()
        print("exitLandning")
        
    def stateLanded(self):
        """
        State: Landed
        Dummy state.
        """
        print("stateLanded")
        return None

    def run(self):
        """Main loop, state machine"""
        try: 
            state = self.stateInit    # initial state
            self.stateNum = uglyConst.S0_INIT
            while state and self.runSM:

                state = state()

        finally:
            #-- Clean up 
            print('Stopping cf_thread')
            if self.cf is not None:
                if self.mc._is_flying: # if cf has not landed, do so
                    self.mc.stop()
                    print('Finally landing')
                    self.mc.land()
                #-- Close link and logging
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
    #cf_thread.start()

    #-- Stopping threads
    cv_thread.join()
    print('cv_thread stopped.')
    #cf_thread.raise_exception()
    #cf_thread.join()
    print('cf_thread stopped.')

        