import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

rows, cols = 1, 1

fig, axs = plt.subplots(rows, cols, sharex=True)
fig.suptitle('Log data')
fig.canvas.manager.full_screen_toggle()
axs.set_title('Battery Voltage')

N = 50


#-- FLY + CAMERA MOUNTED (BLUE)
batFileName = 'bat_log_camoff4.txt'
time_cf, vbat, state, batteryLevel = np.loadtxt(batFileName, delimiter=',', unpack=True)
vbat_conv = np.convolve(vbat, np.ones((N,))/N, mode='same')

#-- Crop 
start = 0
end = len(time_cf)-0
time_cf = time_cf[start:end]/1000
vbat = vbat[start:end]
vbat_conv = vbat_conv[start:end]
time_cf = [(x-time_cf[0]) for x in time_cf] # compensate for system time

#-- Plot
axs.plot(time_cf, vbat, 'tab:blue')
axs.plot(time_cf, vbat_conv, '#0f0f0f')

#-- FLY + CAMERA NOT MOUNTED (GREEN)
batFileName = 'bat_log_camno3.txt'
time_cf, vbat, state, batteryLevel = np.loadtxt(batFileName, delimiter=',', unpack=True)
vbat_conv = np.convolve(vbat, np.ones((N,))/N, mode='same')

#-- Crop 
start = 200
end = len(time_cf)-23
time_cf = time_cf[start:end]/1000
vbat = vbat[start:end]
vbat_conv = vbat_conv[start:end]
time_cf = [(x-time_cf[0]) for x in time_cf] # compensate for system time

#-- Plot
axs.plot(time_cf, vbat, 'tab:green')
axs.plot(time_cf, vbat_conv, '#0f0f0f')

vbat_three = np.full((len(time_cf)),3)
axs.plot(time_cf, vbat_three, '#0f0f0f')

#-- FLY + CAMERA ON (RED)
batFileName = 'bat_log_camon2.txt'
time_cf, vbat, state, batteryLevel = np.loadtxt(batFileName, delimiter=',', unpack=True)
vbat_conv = np.convolve(vbat, np.ones((N,))/N, mode='same')

#-- Crop 
start = 45
end = len(time_cf)-35
time_cf = time_cf[start:end]/1000
vbat = vbat[start:end]
vbat_conv = vbat_conv[start:end]
time_cf = [(x-time_cf[0]) for x in time_cf] # compensate for system time

#-- Plot
axs.plot(time_cf, vbat, 'tab:red')
axs.plot(time_cf, vbat_conv, '#0f0f0f')

# axs[1].set_title('Battery Level')
# axs[1].plot(time_cf, batteryLevel)

# axs[2].set_title('Battery state')
# axs[2].plot(time_cf, state)

plt.show()