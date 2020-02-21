# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.

This example utilizes the SyncCrazyflie and SyncLogger classes.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
open("ugly_log.txt","w")

class UglyLogger:
    def __init__(self,scf):
        self._scf = scf
        self.start_logging(self._scf)

    def log_callback(self, timestamp, data, logconf):
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']
        height = data['range.zrange']
        with open("ugly_log.txt","a") as file:
            file.write("%d,%0.4f,%0.4f,%0.4f,%0.4f\n" % (timestamp,roll,pitch,yaw,height))
        print("WOW")

    def start_logging(self,scf):
        with open("ugly_log.txt","a") as file:
            file.write("timestamp,roll,pitch,yaw,height\n")

        log_conf = LogConfig(name='logdata', period_in_ms=10)
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')
        
        loga_conf.add_variable('range.zrange','float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.log_callback)
        
        log_conf.start()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) == 0:
        print('No Crazyflies found, cannot run example')
    else:
        

        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(available[0][0], cf=cf) as scf:
            lgr = UglyLogger(scf)
            lgr.start_logging(scf)
            endTime = time.time()+1
            while True:

                if time.time() > endTime:
                    break
                
