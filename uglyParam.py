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
Simple example that connects to the first Crazyflie found, triggers
reading of all the parameters and displays their values. It then modifies
one parameter and reads back it's value. Finally it disconnects.
"""
import logging
import random
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class UglyParam:
    """
   
    """

    def __init__(self, cf, link_uri):
        """ Initialize and run the example with the specified link_uri """
        print('CCCOOOOOOOONNNNNNNEEECCCTTEEEDDD')
        self._cf = cf

        # Connect some callbacks from the Crazyflie API

        p_toc = self._cf.param.toc.toc
        for group in sorted(p_toc.keys()):
            print('{}'.format(group))

        self.start_params(link_uri)
        

    def _connected(self, link_uri):
        
        self.start_params(link_uri)

    def start_params(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # You can also register a callback for a specific group.name combo
        self._cf.param.add_update_callback(group='velCtlPid', name=None,
                                           cb=self._velCtrlPid_callback)
        self._cf.param.set_value('velCtlPid.vzKi', 1.0)

    def _velCtrlPid_callback(self, name, value):
        """Specific callback for the velCtlPid group parameters"""
        print('{0} was set to: {1}'.format(name, value))

    def updateGain(self,group,value):
        self._cf.param.set_value(group,value)


# if __name__ == '__main__':
#     # Initialize the low-level drivers (don't list the debug drivers)
#     cflib.crtp.init_drivers(enable_debug_driver=False)
#     # Scan for Crazyflies and use the first one found
#     print('Scanning interfaces for Crazyflies...')
#     available = cflib.crtp.scan_interfaces()
#     print('Crazyflies found:')
#     for i in available:
#         print(i[0])

#     if len(available) > 0:
#         pe = ParamExample(available[0][0])
#         # The Crazyflie lib doesn't contain anything to keep the application
#         # alive, so this is where your application should do something. In our
#         # case we are just waiting until we are disconnected.
#         while pe.is_connected:
#             time.sleep(1)
#     else:
#         print('No Crazyflies found, cannot run example')
