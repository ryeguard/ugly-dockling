import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

from matplotlib.colors import from_levels_and_colors
from matplotlib.collections import LineCollection

from matplotlib import rcParams
plt.rcParams.update({'font.size': 12})
rcParams['axes.titlepad'] = 2

rows, cols = 3, 2

fig, axs = plt.subplots(rows, cols, sharex=True, sharey='col', figsize=(12,7), dpi=100)
#fig.suptitle('Pose data')
now = datetime.datetime.now()
#axs.set_title('Battery Voltage')

N = 50

#fileName = './cv_log.txt'
fileName = '../saved_runs/pose_gaus3/poseTest_2020-04-20_111117.txt'
#fileName = '../saved_runs/pose/poseTest_2020-04-17_1603.txt'


raw_time, x, y, z, roll, pitch, yaw, detected, side = np.loadtxt(fileName, delimiter=',', unpack=True)
raw_data = np.zeros((6,len(raw_time)))

raw_time = [(raw_time[i]-1.0) for i in range(len(raw_time))]

percent_detected = len(raw_time)

# translation 
raw_data[0] = x
raw_data[1] = y
raw_data[2] = z
# rotation about x, y, z
raw_data[3] = roll 
raw_data[4] = pitch
raw_data[5] = yaw

detectarr = np.mod(detected,2) == 1
timeg = np.extract(detectarr, raw_time)
timeb = np.extract(np.invert(detectarr), raw_time)

datag = np.zeros((6,len(timeg)))
datab = np.zeros((6,len(timeb)))

for i in range(6):
    datag[i] = np.extract(detectarr, raw_data[i])
    datab[i] = np.extract(np.invert(detectarr), raw_data[i])

# Statistics
#averages
data_avg = np.average(datag,axis=1)

#std deviation
data_std = np.std(datag,axis=1)

percent_detected = len(timeg)/len(raw_time)
print("Detection rate: ",percent_detected)

side_avg = np.average(side)
print("Marker size [pix]: ", side_avg)

s=1
titles = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
ylabels = ['Distance [m]', 'Distance [m]', 'Distance [m]', 'Angle [deg]', 'Angle [deg]', 'Angle [deg]']
stepsize = [0.05, 0.05, 0.05, 1,1,1]


for i in range(6):
    thisaxs = axs[i%3][int(i/3)]
    thisaxs.scatter(timeg, datag[i], color='g', marker=',', s=s)
    thisaxs.scatter(timeb, datab[i], color='r', marker='.', s=s*100)
    
    #thisaxs.plot((1,11),(data_avg[i],data_avg[i]))
    if i == 2 or i==5:
        thisaxs.set_xlabel('Time [s]')
    thisaxs.set_ylabel(ylabels[i])
    thisaxs.set_title(titles[i])
    start, end = axs[i%3][int(i/3)].get_ylim()
    #axs[i%3][int(i/3)].yaxis.set_ticks(np.arange(start, end, stepsize[i]))

#axs[3][0].plot(raw_time, side)

# Save figure
now = datetime.datetime.now()
date = now.strftime("%Y-%m-%d_%H%M%S")
#plt.savefig('./poseTest_'+date+'.png', dpi=fig.dpi)

# Plot figure
plt.show()


#-- Copy textfile as backup with same name
#shutil.copy(fileName,'./poseTest_'+date+'.txt')


#04171553: 10 cm
#57: 7 cm
#58 20
#59 30
#1600 40
#03 50
#05 60
#21 70
#23   80
#24   90
#26 100
#28 110
#29 120
#31 130
#32 140
#34 140 w angle





