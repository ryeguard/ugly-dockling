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


class UglyLogger:

    def __init__(self, link_uri, scf, f):
        self._cf = Crazyflie(rw_cache='./cache')
        self._scf = scf
        self.start_logging(self._scf)
        self._file = f
        self.start_logging(scf)
        self.batlev = 0.0
        
    def log_callback(self, timestamp, data, logconf):
        vbat = data['pm.vbat']
        state = data['pm.state']
        batteryLevel = data['pm.batteryLevel']
        self.batlev = vbat
        self._file.write("%d,%f,%f,%f\n" % (timestamp,vbat,state,batteryLevel))

    def get_batlev(self):
        return self.batlev

    def start_logging(self,scf):

        log_conf = LogConfig(name='logdata', period_in_ms=100)
        
        #-- Battery voltage
        log_conf.add_variable('pm.vbat', 'float')
        #log_conf.add_variable('pm.vbatMV', 'unint16_t')
        log_conf.add_variable('pm.state', 'int8_t')
        log_conf.add_variable('pm.batteryLevel','uint8_t')

        scf.cf.log.add_config(log_conf)
  
        log_conf.data_received_cb.add_callback(self.log_callback)

        #logz_conf.data_received_cb.add_callback(self.logz_callback)
        
        log_conf.start()