# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
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
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander

# URI to the Crazyflie to connect to
uri = 'radio://0/10/2M/E7E7E7E7E7'

def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf) as motion_commander:
            with PositionHlCommander(
                    scf,
                    x=0.0, y=0.0, z=0.0,
                    default_velocity=0.2,
                    default_height=0.3,
                    controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
                with Multiranger(scf) as multiranger:
                    #init_x, init_y, init_z = pc.get_position()
                    VELOCITY = 0
                    while pc.get_position()[0] < 1:
                        # Obstacle face
                        if (is_close(multiranger.front)):
                            if (is_close(multiranger.right)):
                                print("Left")
                                motion_commander.start_linear_motion(0, VELOCITY, 0)
                            else:
                                print("Right")
                                motion_commander.start_linear_motion(0, -VELOCITY, 0)
                        else:
                            print("Front")
                            motion_commander.start_linear_motion(VELOCITY, 0, 0)
                        time.sleep(0.1)
                        print("Pos x:" + str(pc.get_position()[0]) + " Pos y:" + str(pc.get_position()[1]))
                    pc.land()