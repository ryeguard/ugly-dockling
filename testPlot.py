import matplotlib.pyplot as plt
import numpy as np

with open('logFile.txt', 'r') as infile, open('logFileNew.txt', 'w') as outfile:
    #temp = infile.read().replace(':', "") #'[^a-zA-Z0-9_]'
    temp = infile.read().replace("'", "")
    temp = temp.replace(":", "")
    temp = temp.replace("{", "")
    temp = temp.replace("}", "")
    temp = temp.replace("stabilizer.roll ", "")
    temp = temp.replace("stabilizer.pitch ", "")
    temp = temp.replace("stabilizer.yaw ", "")
    #temp = infile.read().replace(":", "")
    outfile.write(temp)

with open('logz.txt', 'r') as infile, open('logznew.txt', 'w') as outfile:
    #temp = infile.read().replace(':', "") #'[^a-zA-Z0-9_]'
    temp = infile.read().replace("'", "")
    temp = temp.replace(":", "")
    temp = temp.replace("{", "")
    temp = temp.replace("}", "")
    temp = temp.replace("range.zrange ", "")
    outfile.write(temp)

time_stab, roll, pitch, yaw = np.loadtxt('logFileNew.txt', delimiter=',', unpack=True)
time_z, z = np.loadtxt('logznew.txt', delimiter=',', unpack=True)
#plt.plot(time,roll, label='Roll')

fig, axs = plt.subplots(4)
fig.suptitle('IMU- and z-data')
axs[0].plot(time_stab, roll, 'tab:orange')
axs[0].set_title('Roll')
axs[1].plot(time_stab, pitch, 'tab:green')
axs[1].set_title('Pitch')
axs[2].plot(time_stab, yaw, 'tab:red')
axs[2].set_title('Yaw')
axs[3].plot(time_z, z, 'tab:blue')
axs[3].set_title('Height')


#plt.xlabel('time')
#plt.ylabel('Roll')
#plt.title('Interesting Graph\nCheck it out')
#plt.legend()
plt.show()
