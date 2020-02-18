import matplotlib.pyplot as plt
import numpy as np

with open('/home/louise/Documents/crazyflie-lib-python/examples/logTest.txt', 'r') as infile, open('/home/louise/Documents/crazyflie-lib-python/examples/logTestNew.txt', 'w') as outfile:
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

time, roll, pitch, yaw = np.loadtxt('/home/louise/Documents/crazyflie-lib-python/examples/logTestNew.txt', delimiter=',', unpack=True)
#plt.plot(time,roll, label='Roll')

fig, axs = plt.subplots(3)
fig.suptitle('IMU-data')
axs[0].plot(time, roll, 'tab:orange')
axs[0].set_title('Roll')
axs[1].plot(time, pitch, 'tab:green')
axs[1].set_title('Pitch')
axs[2].plot(time, yaw, 'tab:red')
axs[2].set_title('Yaw')
#plt.xlabel('time')
#plt.ylabel('Roll')
#plt.title('Interesting Graph\nCheck it out')
#plt.legend()
plt.show()
