"""
A class that logs the stabilizer and ranger data of a Crazyflie
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# On import, open in write mode to reset file
#open("ugly_log.txt","w")

class UglyLogger:

    def __init__(self, link_uri, scf, f):
        self._cf = Crazyflie(rw_cache='./cache')
        self._scf = scf
        self.start_logging(self._scf)
        self._file = f
        self.start_logging(scf)
        print("logger init")
        
    def log_callback(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        yaw = data['stabilizer.yaw']
        vx = 0 #data['ctrltarget.vx']
        vy = 0 #data['ctrltarget.vy']
        vz = 0 #data['ctrltarget.vz']
        #print("log callback")

        self._file.write("%d,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\n" % (timestamp,x,y,z,yaw,vx,vy,vz))

    def start_logging(self,scf):
        log_conf = LogConfig(name='logdata', period_in_ms=10)
        #-- States
        log_conf.add_variable('stateEstimate.x','float')
        log_conf.add_variable('stateEstimate.y','float')
        log_conf.add_variable('stateEstimate.z','float')
        #log_conf.add_variable('stabilizer.roll', 'float')
        #log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')
        
        #log_conf.add_variable('range.zrange', 'uint16_t')

        #-- Battery voltage
        #log_conf.add_variable('pm.vbat', 'float')

        #-- Command 
        #log_conf.add_variable('ctrltargetZ.vx','float')
        #log_conf.add_variable('ctrltargetZ.vy','float')
        #log_conf.add_variable('ctrltargetZ.vz','float')

        scf.cf.log.add_config(log_conf)
  
        #-- Connect callback
        log_conf.data_received_cb.add_callback(self.log_callback)
        
        log_conf.start()


# if __name__ == '__main__':
#     # Initialize the low-level drivers (don't list the debug drivers)
#     cflib.crtp.init_drivers(enable_debug_driver=False)

#     # Scan for Crazyflies and use the first one found
#     print('Scanning interfaces for Crazyflies...')
#     available = cflib.crtp.scan_interfaces()
#     print('Crazyflies found:')
#     for i in available:
#         print(i[0])

#     if len(available) == 0:
#         print('No Crazyflies found, cannot run example')
#     else:
        

#         cf = Crazyflie(rw_cache='./cache')
#         with open("ugly_log.txt","a") as file:
#             with SyncCrazyflie(available[0][0], cf=cf) as scf:
                

#                 lgr = UglyLogger(available[0][0],scf,file)
#                 #lgr.start_logging(scf)
#                 endTime = time.time()+5
#                 while 1:

#                     if time.time() > endTime:
#                         break
                    
