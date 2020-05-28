import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

#-- Define the file name
cfFileName = 'ugly_log.txt'
cvFileName = 'cv_log.txt'

#with open(cfFileName) as file:
#    next(file)

time_cf, x, y, z, roll, pitch, yaw, height = np.loadtxt(cfFileName, delimiter=',', unpack=True)
time_cv, posx, posy, posz, cvroll, cvpitch, cvyaw, id2follow = np.loadtxt(cvFileName, delimiter=',', unpack=True)  
ti = np.arange(len(time_cv))

time_cf = [x-time_cf[0] for x in time_cf]
height = [x*0.001 for x in height]
yaw = [-1*x+cvyaw[0] for x in yaw]

plt.scatter(posx,posy,c=ti, cmap=cm.hsv_r)
plt.show()

rows, cols = 4, 2

fig, axs = plt.subplots(rows, cols, sharex=True)
fig.suptitle('Log data')

#-- First column [:][0]
axs[0][0].set_title('X')
#axs[0][0].plot(time_cf, x, 'tab:red')
axs[0][0].plot(time_cv, posx, 'tab:green')

axs[1][0].set_title('Y')
#axs[1][0].plot(time_cf, y, 'tab:red')
axs[1][0].plot(time_cv, posy, 'tab:green')

axs[2][0].set_title('Z')
axs[2][0].plot(time_cf, height, 'tab:red')
axs[2][0].plot(time_cv, posz, 'tab:green')
axs[2][0].plot(time_cf, z, 'tab:blue')
#axs[0][0].legend()

axs[3][0].set_title('id2follow')
axs[3][0].plot(time_cv, id2follow, 'tab:blue')

#-- Second column [:][1]
axs[0][1].set_title('Roll')
axs[0][1].plot(time_cv, -cvpitch, 'tab:green')
axs[0][1].plot(time_cf, roll, 'tab:red')

axs[1][1].set_title('Pitch')
axs[1][1].plot(time_cv, cvroll, 'tab:green')
axs[1][1].plot(time_cf, pitch, 'tab:red')

axs[2][1].set_title('Yaw')
axs[2][1].plot(time_cv, cvyaw, 'tab:green')
axs[2][1].plot(time_cf, yaw, 'tab:red')

axs[3][1].set_title('id2follow')
axs[3][1].plot(time_cv, id2follow, 'tab:blue')

#-- Third column [:][2]
# axs[0][2].set_title('u_x')
# axs[0][2].plot(time_cf, vx, 'tab:blue')

# axs[1][2].set_title('u_y')
# axs[1][2].plot(time_cf, vy, 'tab:blue')

# axs[3][2].set_title('id2follow')
# axs[3][2].plot(time_cv, id2follow, 'tab:blue')

#plt.xlabel('time')
#plt.ylabel('Roll')
#plt.title('Interesting Graph\nCheck it out')
#plt.legend()

#-- Save figure as png
now = datetime.datetime.now()
date = now.strftime("%Y-%m-%d_%H%M")
plt.savefig('figures/uglyPlotter_'+date+'.png')

#-- Copy textfile as backup with same name
shutil.copy(cfFileName,'figures/cf_log'+date+'.txt')
shutil.copy(cvFileName,'figures/cv_log'+date+'.txt')

plt.show()

# with open('logFile.txt', 'r') as infile, open('logFileNew.txt', 'w') as outfile:
#     #temp = infile.read().replace(':', "") #'[^a-zA-Z0-9_]'
#     temp = infile.read().replace("'", "")
#     temp = temp.replace(":", "")
#     temp = temp.replace("{", "")
#     temp = temp.replace("}", "")
#     temp = temp.replace("stabilizer.roll ", "")
#     temp = temp.replace("stabilizer.pitch ", "")
#     temp = temp.replace("stabilizer.yaw ", "")
#     #temp = infile.read().replace(":", "")
#     outfile.write(temp)

# with open('logz.txt', 'r') as infile, open('logznew.txt', 'w') as outfile:
#     #temp = infile.read().replace(':', "") #'[^a-zA-Z0-9_]'
#     temp = infile.read().replace("'", "")
#     temp = temp.replace(":", "")
#     temp = temp.replace("{", "")
#     temp = temp.replace("}", "")
#     temp = temp.replace("range.zrange ", "")
#     outfile.write(temp)