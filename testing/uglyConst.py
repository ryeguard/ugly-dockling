#-- CONSTANTS FOR UGLY-DOCKLING
# All time is in seconds. 
# All distances/velocities are in meters/meters per second.

#-- OpenCV settings
CAM_NR = 2
MARKER_OFFSET = 0.08935
MARKERSIZE_SMALL = 0.0215
MARKERSIZE_BIG = 0.112
MARKERID_BIG = 0 #10
MARKERID_SMALL = 1 #17

#-- OpenCV colors
BLUE = (255,0,0)
GREEN = (0,255,0)
RED = (0,0,255)
BLACK = (0,0,0)
WHITE = (255,255,255)

#-- State machine
TAKEOFF_HEIGHT = 0.8
TAKEOFF_ZVEL = 0.5 
APPROACH_ZVEL = 0.1
FAR_DIST = 0.025
FAR_ANGL = 1.0

FAR_CONE = 3.0

POSE_HEIGHT = 0.4

LANDING_HEIGHT = 0.05
LANDING_ZVEL = 0.05
LANDING_DIST = 0.005

DIST_IGE = 0.15
DIST_IGE_HYST = 0.05
R_CYLINDER = 0.05
R_CYLINDER_HYST = 0.04     

#-- Control parameters
CTRL_NONE = 0           # no landing control, just vertical velocity=LANDING_ZVEL
CTRL_POSD = 1
CTRL_PIX = 2

Kyaw = 0.5
Kx = 0.2
Ky = 0.2
Kz = 1.0

PIX_Kx = 0.001
PIX_Ky = 0.001
PIX_Kz = 5.0

velKp = 25.0
velKi = 15.0
velKd = 0.0

MAX_XVEL = 1.0
MAX_YVEL = 1.0
MAX_ZVEL = 1.0

