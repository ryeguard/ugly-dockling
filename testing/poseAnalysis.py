import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

from matplotlib.colors import from_levels_and_colors
from matplotlib.collections import LineCollection

import glob, os

rows, cols = 4,2

fileNum = 0
fig, axs = plt.subplots(rows, cols, sharex=True)
fig.suptitle('Pose data')
fig.canvas.manager.full_screen_toggle()
now = datetime.datetime.now()
path = "../saved_runs/pose"

def analyse_run(path, fig, axs, color):
    fileNum = 0
    os.chdir(path)
    for file in glob.glob("*.txt"):
        fileNum += 1

    print("Number of files: ",fileNum)
    data_avg = np.zeros((fileNum,7))
    data_std = np.zeros((fileNum,7))
    cov = np.zeros((fileNum,7))
    percent_detected = np.zeros((fileNum,1))

    fileNum = 0
    for file in sorted(glob.glob("*.txt")):
        fileName = str(file)

        raw_time, x, y, z, roll, pitch, yaw, detected, side = np.loadtxt(fileName, delimiter=',', unpack=True)
        raw_data = np.zeros((7,len(raw_time)))

        # translation 
        raw_data[0] = x
        raw_data[1] = y
        raw_data[2] = z
        # rotation about x, y, z
        raw_data[3] = roll 
        raw_data[4] = pitch
        raw_data[5] = yaw
        # marker size
        raw_data[6] = side

        detectarr = np.mod(detected,2) == 1
        timeg = np.extract(detectarr, raw_time)
        timeb = np.extract(np.invert(detectarr), raw_time)

        datag = np.zeros((7,len(timeg)))
        datab = np.zeros((7,len(timeb)))

        for i in range(7):
            datag[i] = np.extract(detectarr, raw_data[i])
            datab[i] = np.extract(np.invert(detectarr), raw_data[i])

        # Statistics
        #averages
        data_avg[fileNum] = np.mean(datag,axis=1)

        #std deviation
        data_std[fileNum] = np.std(datag,axis=1)
        cov[fileNum] = np.divide(data_std[fileNum],np.abs(data_avg[fileNum]))
        
        percent_detected[fileNum] = len(timeg)/len(raw_time)
        #print("Detection rate: ",percent_detected[fileNum])

        #side_avg[fileNum] = np.average(side)
        #print("Marker size [pix]: ", side_avg)

        s=1

        # for i in range(6):
        #     axs[i%3][int(i/3)].scatter(timeg, datag[i], color='g', marker=',', s=s)
        #     axs[i%3][int(i/3)].scatter(timeb, datab[i], color='r', marker='.', s=s*100)
            
        #     axs[i%3][int(i/3)].plot((1,11),(data_avg[i],data_avg[i]))

        # axs[3][0].plot(raw_time, side)

        fileNum += 1

    #print(data_avg.shape)
    print(data_avg[:,6])

    lengths = np.array((7,10,20,30,40,50,60,70,80,90,100,110,120,130,140))

    ylim = np.array((1,2,0.1,1,0.5,0.5,1))

    for i in range(6): 
        r,c = i%3, int(i/3)
        #axs[r][c].set_ylim(0,ylim[i])
        axs[r][c].plot((data_avg[:,6]),cov[:,i], c=color)
        #axs[r][c].plot((data_avg[:,6]),data_std[:,i], c='r')

    axs[3][0].plot((data_avg[:,6]),cov[:,6], c=color)
    plt.xlim(350, 0)
    # Save figure
    now = datetime.datetime.now()
    date = now.strftime("%Y-%m-%d_%H%M")
#plt.savefig('./poseTest_'+date+'.png', dpi=fig.dpi)

analyse_run(path="../saved_runs/pose", fig=fig, axs=axs, color='g')
analyse_run(path="../pose_gaus3", fig=fig, axs=axs, color='r')
analyse_run(path="../pose_gaus5", fig=fig, axs=axs, color='b')

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
#06 60
#21 70
#23   80
#24   90
#26 100
#28 110
#29 120
#31 130
#32 140
#34 140 w angle





