#-- 

#-- OpenCV settings
CAM_NR = 2
MARKER_OFFSET = 0.08935
MARKERSIZE_SMALL = 0.0215
MARKERSIZE_BIG = 0.112

#-- OpenCV colors
BLUE = (255,0,0)
GREEN = (0,255,0)
RED = (0,0,255)
BLACK = (0,0,0)
WHITE = (255,255,255)

#-- State machine
TAKEOFF_HEIGHT = 0.5   
TAKEOFF_ZVEL = 0.5 
APPROACH_ZVEL = 0.05
FAR_DIST = 0.05
FAR_ANGL = 1.0

LANDING_HEIGHT = 0.1

DIST_IGE = 0.2          # [m]
DIST_IGE_HYST = 0.01
R_CYLINDER = 0.05       # radius of x,y [m]
R_CYLINDER_HYST = 0.04     

#-- Control parameters
Kyaw = 1.0
Kx = 0.2
Ky = 0.2
Kz = 40

velKp = 25.0
velKi = 15.0
velKd = 0.0

MAX_XVEL = 1.0
MAX_YVEL = 1.0
MAX_ZVEL = 1.0

