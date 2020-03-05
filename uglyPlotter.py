import matplotlib.pyplot as plt
import datetime
import shutil
import numpy as np

#-- Define the file name
fileName = 'ugly_log.txt'

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

with open(fileName) as file:
    next(file)

time_stab, roll, pitch, yaw, height, heightRead = np.loadtxt(fileName, delimiter=',', unpack=True)
#time_z, z = np.loadtxt('logznew.txt', delimiter=',', unpack=True)
#plt.plot(time,roll, label='Roll')

fig, axs = plt.subplots(5)
fig.suptitle('IMU- and z-data')
axs[0].plot(time_stab, roll, 'tab:orange')
axs[0].set_title('Roll')
axs[1].plot(time_stab, pitch, 'tab:green')
axs[1].set_title('Pitch')
axs[2].plot(time_stab, yaw, 'tab:red')
axs[2].set_title('Yaw')
axs[3].plot(time_stab, height, 'tab:blue')
axs[3].set_title('Height')
axs[4].plot(time_stab, heightRead, 'tab:blue')
axs[4].set_title('HeightRead')

#plt.xlabel('time')
#plt.ylabel('Roll')
#plt.title('Interesting Graph\nCheck it out')
#plt.legend()

#-- Save figure as png
now = datetime.datetime.now()
date = now.strftime("%Y-%m-%d_%H:%M")
plt.savefig('figures/uglyPlotter_'+date+'.png')

#-- Copy textfile as backup with same name
shutil.copy(fileName,'figures/uglyPlotter_'+date+'.txt')

plt.show()