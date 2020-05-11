import matplotlib.pyplot as plt
import matplotlib as lib
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np
import uglyConst

timeBeforeStep = 2000
timeAfterStep = 6000

#-- Define the file name
#controllerTest_cf_log_8-10_2020-05-11_153154.txt : ang
#controllerTest_cf_log_8-10_2020-05-11_153320.txt : dist
#controllerTest_cf_log_8-10_2020-05-11_153524.txt : dist
cfFileName = 'controllerTest_cf_log_8-10_2020-05-11_153154.txt'
cvFileName = 'controllerTest_cv_log_8-10_2020-05-11_153154.txt'

time_cv, cvx, cvy, cvz, cvroll, cvpitch, cvyaw, id2follow, hasStep = np.loadtxt(cvFileName, delimiter=',', unpack=True)  
time_cf, cfx, cfy, cfz, cfyaw, cfvx, cfvy, cfvyaw = np.loadtxt(cfFileName, delimiter=',', unpack=True)

numCvPoints = len(time_cv)
numCfPoints = len(time_cf)

stepInd = np.min(np.where(hasStep==1))
stepTime = time_cv[stepInd]

# --- Mask data ---
# CV
mask1 = np.where(time_cv>stepTime-timeBeforeStep)
time_cv = np.array(time_cv)[mask1]
mask2 = np.where(time_cv<stepTime+timeAfterStep)
time_cv = np.array(time_cv)[mask2]

cvx = np.array(cvx)[mask1]
cvx = np.array(cvx)[mask2]
cvy = np.array(cvy)[mask1]
cvy = np.array(cvy)[mask2]
cvyaw = np.array(cvyaw)[mask1]
cvyaw = np.array(cvyaw)[mask2]
hasStep = np.array(hasStep)[mask1]
hasStep = np.array(hasStep)[mask2]


# CF
time_cf = [(x-time_cf[0]) for x in time_cf]
mask3 = np.where(time_cf>stepTime-timeBeforeStep)
time_cf = np.array(time_cf)[mask3]
mask4 = np.where(time_cf<stepTime+timeAfterStep)
time_cf = np.array(time_cf)[mask4]

cfx = np.array(cfx)[mask3]
cfx = np.array(cfx)[mask4]
cfy = np.array(cfy)[mask3]
cfy = np.array(cfy)[mask4]
cfvx = np.array(cfvx)[mask3]
cfvx = np.array(cfvx)[mask4]
cfvy = np.array(cfvy)[mask3]
cfvy = np.array(cfvy)[mask4]
cfyaw = np.array(cfyaw)[mask3]
cfyaw = np.array(cfyaw)[mask4]
cfvyaw = np.array(cfvyaw)[mask3]
cfvyaw = np.array(cfvyaw)[mask4]

#
ti = np.arange(len(time_cv))
numCvPoints = len(time_cv)
numCfPoints = len(time_cf)
time_cf = [(x - time_cf[0])/1000.0 for x in time_cf]
time_cv = [(x - time_cv[0])/1000.0 for x in time_cv]

#cfyaw = [-1*x+cvyaw[0] for x in cfyaw]
#cvyaw = [-x for x in cvyaw]
#cfx = [-1*(i)+cvx[0] for i in cfx]
#cfy = [-1*(i)+cvy[0] for i in cfy]

# Calculate distance and speed
cvvx = np.zeros((numCvPoints,1))
cvvy = np.zeros((numCvPoints,1))

for i in range(0,numCvPoints):
    if hasStep[i] == 0:
        cvvx[i] = (cvx[i]-0.707106/2)*uglyConst.Kx
        cvvy[i] = (cvx[i]-0.707106/2)*uglyConst.Ky
    else: 
        cvvx[i] = cvx[i]*uglyConst.Kx
        cvvy[i] = cvx[i]*uglyConst.Ky

cvr = [(np.hypot(cvx[i],cvy[i])) for i in range(0,numCvPoints)]
cvvr = [(np.hypot(cvvx[i],cvvy[i])) for i in range(0,numCvPoints)]
cfvr = [(np.hypot(cfvx[i],cfvy[i])) for i in range(0,numCfPoints)]

# Calculate command angle vyaw
temp = (np.pi * 1000.0) / 180.0
cfvyaw = [int(x)/temp for x in cfvyaw]
cvvyaw = np.zeros((numCvPoints,1))

for i in range(0,numCvPoints):
    if hasStep[i] == 0:
        cvvyaw[i] = (cvyaw[i]+135)*uglyConst.Kyaw
    else:
        cvvyaw[i] = (cvyaw[i])*uglyConst.Kyaw


def plotStepDist():
    ref = [-0.5*(hasStep[i]-1) for i in range(0,numCvPoints)]

    # Scatter plot
    fig, ax = plt.subplots()
    ax.scatter(cvx,cvy,c=ti, cmap=cm.plasma)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_xlim((-0.15,0.5))
    ax.set_ylim((-0.15,0.5))
    x0,x1 = ax.get_xlim()
    y0,y1 = ax.get_ylim()
    ax.set_aspect(abs(x1-x0)/abs(y1-y0))
    ax.axvline(c='grey', lw=1)
    ax.axhline(c='grey', lw=1)
    c = lib.patches.Circle((0,0), radius = 0.5, ec='grey', fill=False, lw=1)
    ax.add_patch(c)
    plt.show()

    # Step response
    rows, cols = 2,1
    fig, axs = plt.subplots(rows, cols, sharex=True)

    #-- First column [:][0]
    axs[0].plot(time_cv, cvr, 'tab:green')
    axs[0].plot(time_cv, ref, 'tab:red')
    axs[0].set_ylabel('Distance [m]')

    axs[1].plot(time_cf, cfvr, 'tab:blue')
    axs[1].plot(time_cv, cvvr, 'tab:green')
    axs[1].set_ylabel('Command signal [m/s]')

    plt.xlabel('Time [s]')
    plt.xlim(0,(timeBeforeStep+timeAfterStep)/1000.0)

    plt.show()

def plotStepAng():
    ref = [-135*(hasStep[i]-1) for i in range(0,numCvPoints)]

    rows, cols = 2,1
    fig, axs = plt.subplots(rows, cols, sharex=True)

    axs[0].plot(time_cv, -cvyaw, 'tab:green')
    axs[0].plot(time_cv, ref, 'tab:red')
    axs[0].set_ylabel('Angle [deg]')

    axs[1].plot(time_cf, cfvyaw, 'tab:blue')
    axs[1].plot(time_cv, cvvyaw, 'tab:green')
    axs[1].set_ylabel('Command signal [degs/s]')

    plt.xlabel('Time [s]')
    plt.xlim(0,(timeBeforeStep+timeAfterStep)/1000.0)

    plt.show()


plotStepAng()

#-- Save figure as png
now = datetime.datetime.now()
date = now.strftime("%Y-%m-%d_%H%M%S")
#plt.savefig('../figures/controlx_'+date+'.png')

#-- Copy textfile as backup with same name
#shutil.copy(cfFileName,'./controllerTest_cf_log'+'_'+str(int(uglyConst.Kx*10))+'-'+str(int(uglyConst.Kyaw*10))+'_'+date+'.txt')
#shutil.copy(cvFileName,'./controllerTest_cv_log'+'_'+str(int(uglyConst.Kx*10))+'-'+str(int(uglyConst.Kyaw*10))+'_'+date+'.txt')

