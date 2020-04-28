import matplotlib.pyplot as plt
import matplotlib.cm as cm
import datetime
import shutil
import numpy as np

rows, cols = 1, 1

fig, axs = plt.subplots(rows, cols, sharex=True, figsize=(12,7), dpi=80)
#fig.suptitle('Log data')
#fig.canvas.manager.full_screen_toggle()
axs.set_title('Battery life')

N = 50

#-- FLY + CAMERA MOUNTED (BLUE)
batFileName = '../saved_runs/battery/bat_log_camoff4.txt'
time_cf, vbat, state, batteryLevel = np.loadtxt(batFileName, delimiter=',', unpack=True)
vbat_conv = np.convolve(vbat, np.ones((N,))/N, mode='same')

#-- Crop 
start = 50
end = len(time_cf)-30
time_cf = time_cf[start:end]/1000
vbat = vbat[start:end]
vbat_conv = vbat_conv[start:end]
time_cf = [(x-time_cf[0]) for x in time_cf] # compensate for system time

#-- Plot
axs.plot(time_cf, vbat, 'tab:blue', label='Camera off')
axs.plot(time_cf, vbat_conv, '#0f0f0f')

#-- FLY + CAMERA NOT MOUNTED (GREEN)
batFileName = '../saved_runs/battery/bat_log_camno3.txt'
time_cf, vbat, state, batteryLevel = np.loadtxt(batFileName, delimiter=',', unpack=True)
vbat_conv = np.convolve(vbat, np.ones((N,))/N, mode='same')

#-- Crop 
start = 200
end = len(time_cf)-30
time_cf = time_cf[start:end]/1000
vbat = vbat[start:end]
vbat_conv = vbat_conv[start:end]
time_cf = [(x-time_cf[0]) for x in time_cf] # compensate for system time

#-- Plot
axs.plot(time_cf, vbat, 'tab:green', label='No camera')
axs.plot(time_cf, vbat_conv, '#0f0f0f')

vbat_three = np.full((len(time_cf)),3)
axs.plot(time_cf, vbat_three, '#0f0f0f')

#-- FLY + CAMERA ON (RED)
batFileName = '../saved_runs/battery/bat_log_camon2.txt'
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
axs.plot(time_cf, vbat, 'tab:red', label='Camera on')
axs.plot(time_cf, vbat_conv, '#0f0f0f')

plt.xlabel('Duration [s]')
plt.ylabel('Battery voltage [V]')
plt.xticks(np.arange(0, 451, 30)) 
plt.yticks(np.arange(3.0, 4, 0.1)) 
axs.legend()

plt.show()
