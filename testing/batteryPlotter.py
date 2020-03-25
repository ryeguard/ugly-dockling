import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

batFileName = 'bat_log_onlyuav.txt'

time_cf, vbat, state, batteryLevel = np.loadtxt(batFileName, delimiter=',', unpack=True)

rows, cols = 3, 1

fig, axs = plt.subplots(rows, cols, sharex=True)
fig.suptitle('Log data')

axs[0].set_title('Battery Voltage')
axs[0].plot(time_cf, vbat, 'tab:green')

axs[1].set_title('Battery Level')
axs[1].plot(time_cf, batteryLevel)

axs[2].set_title('Battery state')
axs[2].plot(time_cf, state)

plt.show()