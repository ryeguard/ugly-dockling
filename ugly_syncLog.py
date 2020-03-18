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
open("ugly_log.txt","w")
open("ugly_logz.txt","w")


class UglyLogger:

    def __init__(self, link_uri, scf, f):
        self._cf = Crazyflie(rw_cache='./cache')
        self._scf = scf
        self.start_logging(self._scf)
        self._file = f
        self._latestHeight = 0.0
        self._heightRead = 0
    
        self.start_logging(scf)
        
    def log_callback(self, timestamp, data, logconf):
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        height = data['range.zrange'] # [mm]
        self._latestHeight = height
        self._file.write("%d,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\n" % (timestamp,roll,pitch,yaw,height,self._heightRead))


    # def logz_callback(self, timestamp, data, logconf):
    #     height = data['range.zrange']
    #     #file = open("ugly_logz.txt","a")
    #     self._file.write("%d,%0.4f\n" % (timestamp,height))
    #     #file.close()

    #     #with open("ugly_logz.txt","a") as file:
    #     #    file.write("%d,%0.4f\n" % (timestamp,height))
    #     #print("WOOOW")

    def start_logging(self,scf):
        #with open("ugly_log.txt","a") as file:
        #    file.write("timestamp,roll,pitch,yaw,height\n")

        log_conf = LogConfig(name='logdata', period_in_ms=10)
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')
        log_conf.add_variable('range.zrange', 'uint16_t')
    
        #param_conf.add_variable('posCtl.VZp')
        #param_conf.add_variable('posCtl.VZi')
        #param_conf.add_variable('posCtl.VZd')

        #logz_conf = LogConfig(name='logzdata', period_in_ms=25)
        #logz_conf.add_variable('range.zrange','float')

        scf.cf.log.add_config(log_conf)
  
        log_conf.data_received_cb.add_callback(self.log_callback)

        #logz_conf.data_received_cb.add_callback(self.logz_callback)
        
        log_conf.start()

        #logz_conf.start()

    def getHeight(self):
        if self._heightRead == 0:
            self._heightRead = 1
        else:
            self._heightRead = 0
        return self._latestHeight/1000 # convert to m


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
                    
