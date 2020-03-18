import matplotlib.pyplot as plt
import datetime
import shutil
import numpy as np

#-- Define the file name
fileName = 'thrust_log.txt'

with open(fileName) as file:
    next(file)

time_stab, height, thrust, pwm1, pwm2, pwm3, pwm4 = np.loadtxt(fileName, delimiter=',', unpack=True)

fig, axs = plt.subplots(3)
fig.suptitle('Thrust/Height data')
axs[0].plot(time_stab, height, 'tab:orange')
axs[0].set_title('Height')
axs[1].plot(time_stab, thrust, 'tab:green')
axs[1].set_title('Thrust')

axs[2].plot(time_stab, pwm1, time_stab, pwm2, time_stab, pwm3, time_stab, pwm4, 'tab:green')
axs[2].set_title('PWMs')

#plt.xlabel('time')
#plt.ylabel('Roll')
#plt.title('Interesting Graph\nCheck it out')
#plt.legend()

#-- Save figure as png
now = datetime.datetime.now()
date = now.strftime("%Y-%m-%d_%H%M")
plt.savefig('thrustPlotter_'+date+'.png')

#-- Copy textfile as backup with same name
shutil.copy(fileName,'thrustPlotter_'+date+'.txt')

plt.show()