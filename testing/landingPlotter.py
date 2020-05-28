import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np
from scipy.ndimage.interpolation import rotate

# Import data
fileName = 'landingLog.txt'   #id,time,x,y,yaw,markerDetect,mx,my,myaw
runid,time,x,y,yaw,markerDetect,mx,my,myaw = np.loadtxt(fileName, delimiter=',', unpack=True)

# Variables 
numRuns = len(runid)
ampFactor = 2.0

#Fusion
x_fus = np.zeros(numRuns,dtype=np.float64)
y_fus = np.zeros(numRuns,dtype=np.float64)
yaw_fus = np.zeros(numRuns,dtype=np.float64)

for i in range(numRuns):
    if markerDetect[i]:
        x_fus[i] = (mx[i]*1000+x[i]*10)/2.0
        y_fus[i] = (my[i]*1000+y[i]*10)/2.0
        yaw_fus[i] = (myaw[i]+yaw[i])/2.0
    else: 
        x_fus[i] = x[i]*10
        y_fus[i] = y[i]*10
        yaw_fus[i] = yaw[i]

# Statistics
yaw_abs = np.absolute(yaw_fus)
yaw_sum = np.sum(yaw_abs)
yaw_avg = np.average(yaw_abs)
yaw_std = np.std(yaw_abs)

#print('Avg yaw: '+str(np.average(yaw_fus)))
#print('Std yaw: '+str(yaw_std))

r = [(np.hypot(x_fus[i],y_fus[i])) for i in range(0,numRuns)]
r_avg = np.average(r)
r_std = np.std(r)

#print('Avg radius: '+str(r_avg))
#print('Std radius: '+str(r_std))

#print('Percent detect: '+str(np.sum(markerDetect)/numRuns))

# Define figure
rows, cols = 1, 1
fig, axs = plt.subplots(rows, cols, sharex=True, figsize=(7,7), dpi=80)
xlim, ylim = 50, 50
plt.xlim(-xlim, xlim)
plt.ylim(-ylim, ylim)
plt.xticks(np.arange(-xlim+10, xlim, 10)) 
plt.yticks(np.arange(-ylim+10, ylim, 10)) 
#axs.set_title('Landing accuracy')
plt.xlabel('x [mm]')
plt.ylabel('y [mm]')

# Define colors and markers
grey = '#f0f0f0'
black = '#000000'

# Define markers
s = 50 # size

# Draw circles
c = matplotlib.patches.Circle((0,0), radius = 0.1, ec=black, fill=False, lw=1)
axs.add_patch(c)
for i in range(1,5):
    label = "r="+str(i*10)
    plt.text(i*10+1,0,label,fontsize=8)
    c = matplotlib.patches.Circle((0,0), radius = i*10, ec=black, fill=False, lw=.5)
    axs.add_patch(c)

# Plot landing pose
for i in range(numRuns):
    if markerDetect[i] == 1:
        axs.scatter(x_fus[i],y_fus[i], s=s, marker="o")
    else:
        axs.scatter(x_fus[i],y_fus[i], s=s, marker="o", fc='none', edgecolors=black)
    axs.scatter(x_fus[i],y_fus[i], c=black, s=s*2, marker=(2,2,yaw_fus[i]*ampFactor))


#axs.legend()

#plt.show()

# Calculate drone displacement
dr = np.zeros((numRuns,1))

OA = np.array([0.0, -28.0])
for i in range(0,numRuns):
    ang = yaw_fus[i]
    #print('Angle: ',ang)
    #print('Translation: ',np.array([x_fus[i],y_fus[i]]))
    ang = np.deg2rad(ang)
    rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    OD1 = np.matmul(OA,rot)
    #print('OD before t',OD1)
    OD2 = OD1+np.array([x_fus[i],y_fus[i]]) # OD1 + OC
    #print('OD after t',OD2)
    DC = OA - OD2
    dr[i] = np.hypot(DC[0], DC[1])
    
print(dr) 