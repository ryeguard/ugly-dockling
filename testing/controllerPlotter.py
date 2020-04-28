import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

#-- Define the file name
cfFileName = 'cf_log.txt'
cvFileName = 'cv_log.txt'

time_cf, cfx, cfy, cfz, cfyaw, vx, vy, vz = np.loadtxt(cfFileName, delimiter=',', unpack=True)
time_cv, cvx, cvy, cvz, cvroll, cvpitch, cvyaw, id2follow = np.loadtxt(cvFileName, delimiter=',', unpack=True)  
ti = np.arange(len(time_cv))

numCvPoints = len(time_cv)

time_cf = [x-time_cf[0] for x in time_cf]
cfyaw = [-1*x+cvyaw[0] for x in cfyaw]
cfx = [-1*(i)+cvx[0] for i in cfx]
cfy = [-1*(i)+cvy[0] for i in cfy]

plt.scatter(cvx,cvy,c=ti, cmap=cm.hsv_r)
plt.show()

rows, cols = 3,1

#
r = [(np.hypot(cvx[i],cvy[i])) for i in range(0,numCvPoints)]


fig, axs = plt.subplots(rows, cols, sharex=True)
fig.suptitle('Controller log')

#-- First column [:][0]
axs[0].set_title('X')
axs[0].plot(time_cf, cfx, 'tab:red')
axs[0].plot(time_cv, cvx, 'tab:green')

axs[1].set_title('Y')
axs[1].plot(time_cf, cfy, 'tab:red')
axs[1].plot(time_cv, cvy, 'tab:green')

axs[2].set_title('r')
axs[2].plot(time_cv, r, 'tab:green')

#-- Save figure as png
now = datetime.datetime.now()
date = now.strftime("%Y-%m-%d_%H%M")
#plt.savefig('../figures/controlx_'+date+'.png')

#-- Copy textfile as backup with same name
#shutil.copy(cfFileName,'../figures/cf_log'+date+'.txt')
#shutil.copy(cvFileName,'../figures/cv_log'+date+'.txt')

plt.show()