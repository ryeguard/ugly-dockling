# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
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

"""
import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logging.basicConfig(level=logging.ERROR)

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._file = open("thrust_log.txt","w+")
        self._cf.open_link(link_uri)
        self.is_connected = True

        print('Connecting to %s' % link_uri)

    def _motorPowerSet_callback(self, name, value):
        print('{0} was set to: {1}'.format(name, value))

    def updatePWM(self,value):
        self._cf.param.set_value('motorPowerSet.m1',value)
        self._cf.param.set_value('motorPowerSet.m2',value)
        self._cf.param.set_value('motorPowerSet.m3',value)
        self._cf.param.set_value('motorPowerSet.m4',value)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected")
        try:
            self._lg = LogConfig(name='LogData', period_in_ms=10)
            self._lg.add_variable('range.zrange', 'uint16_t')
            self._lg.add_variable('stabilizer.thrust', 'uint16_t')
            self._lg.add_variable('pwm.m1_pwm', 'uint32_t')
            self._lg.add_variable('pwm.m2_pwm', 'uint32_t')
            self._lg.add_variable('pwm.m3_pwm', 'uint32_t')
            self._lg.add_variable('pwm.m4_pwm', 'uint32_t')
            
            print("Added variables and opened file.")
            self._cf.log.add_config(self._lg)
            self._lg.data_received_cb.add_callback(self.log_callback)
            self._lg.start()

        except:
            print("ERROR")

        self._cf.param.set_value('motorPowerSet.enable',1)
        self._cf.param.add_update_callback(group='motorPowerSet', name=None, cb=self._motorPowerSet_callback)


        print("Started logging")
        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

    def log_callback(self, timestamp, data, logconf):
            height = data['range.zrange']
            thrust = data['stabilizer.thrust']
            pwm1 = data['pwm.m1_pwm']
            pwm2 = data['pwm.m2_pwm']
            pwm3 = data['pwm.m3_pwm']
            pwm4 = data['pwm.m4_pwm']

            self._file.write("%d,%0.4f,%4.4f,%d,%d,%d,%d\n" % (timestamp,height,thrust,pwm1,pwm2,pwm3,pwm4))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 1000
        thrust = 1000 # 0 - 65535
        pitch = 0
        roll = 0
        yawrate = 0
        pwm_mult = 1
        pwm_step = int(65535/10)
        pwm = 0
        pwm_max = int(65535 - 65535/10)

        # Unlock startup thrust protection
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        self.updatePWM(0)


        # while thrust >= 0:
        #     print("Ramping"+str(thrust))
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     time.sleep(0.1)
        #     if thrust >= 10000:
        #         thrust_mult = -1
        #     thrust += thrust_step * thrust_mult
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(1)
        while pwm >= 0:
            #print("Ramping "+str(pwm))
            self.updatePWM(pwm)
            time.sleep(5)
            if pwm > pwm_max:
                pwm_mult = -100000
            pwm += pwm_step*pwm_mult
        self.updatePWM(0)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()
        self._file.close()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = MotorRampExample(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')

    while le.is_connected:
        time.sleep(1)
