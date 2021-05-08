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
import sys
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander

# URI to the Crazyflie to connect to
uri = 'radio://0/10/2M/E7E7E7E7E7'
x0 = 0
y0 = 0
ADVANCE = 1
GOAL = 2
CORNER = 3

if len(sys.argv) >1:
    x0 = float(sys.argv[1])
    if len(sys.argv) >2:
        y0 = float(sys.argv[2])

def get_off_U(x,y,VELOCITY = 0.2):
    while is_close(multiranger.right) and is_close(multiranger.left):
        motion_commander.start_linear_motion(-VELOCITY, 0, 0)
        time.sleep(0.1)
        x -= VELOCITY * 0.1
    if not is_close(multiranger.right) and not is_close(multiranger.left):
        if y < 1.5:
            motion_commander.start_linear_motion(0, VELOCITY, 0)
            time.sleep(0.5)
            y += VELOCITY * 0.5
        else:
            motion_commander.start_linear_motion(0, -VELOCITY, 0)
            time.sleep(0.5)
            y -= VELOCITY * 0.5
    elif not is_close(multiranger.right):
        motion_commander.start_linear_motion(0, -VELOCITY, 0)
        time.sleep(0.5)
        y -= VELOCITY * 0.5
    else:
        motion_commander.start_linear_motion(0, VELOCITY, 0)
        time.sleep(0.5)
        y += VELOCITY * 0.5
    return x,y


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        with MotionCommander(
                scf,
                default_height = 0.2) as motion_commander:
            with Multiranger(scf) as multiranger:
                VELOCITY = 0.2
                keep_flying = True
                x = x0
                y = y0
                STATE = ADVANCE
                time.sleep(1)
                direction = 0
                line0 = False
                line3 = True
                while keep_flying:
                    vx = 0
                    vy = 0
                    # Main loop of the controller
                    if x > 3.6:
                        STATE = GOAL
                        
                    if STATE == ADVANCE:
                        if (is_close(multiranger.front)):
                            if is_close(multiranger.right) and is_close(multiranger.left):
                                print("U")
                                (x,y) = get_off_U(x,y)
                            elif (is_close(multiranger.right)):
                                print("Left")
                                if y > 2.8:
                                    STATE = CORNER
                                else:
                                    vy = VELOCITY
                            elif (is_close(multiranger.left)):
                                print("Right")
                                if y < 0.2:
                                    STATE = CORNER
                                else :
                                    vy = -VELOCITY
                            else:
                                if y < 1.5 and direction == 0:
                                    vy = VELOCITY
                                    direction = 1
                                elif direction == 0:
                                    vy = -VELOCITY
                                    direction = -1
                                else:
                                    vy = VELOCITY * direction
                        else:
                            print("Front")
                            vx = VELOCITY
                            direction = 0
                    elif STATE == CORNER:
                        if y < 1.5:
                            if (is_close(multiranger.left)):
                                vx = -VELOCITY
                            else:
                                motion_commander.start_linear_motion(0, VELOCITY, 0)
                                time.sleep(0.5)
                                y += VELOCITY * 0.5
                                if not(is_close(multiranger.front)):
                                    STATE = ADVANCE
                        else:
                            if (is_close(multiranger.right)):
                                vx = -VELOCITY
                            else:
                                motion_commander.start_linear_motion(0, -VELOCITY, 0)
                                time.sleep(0.5)
                                y -= VELOCITY * 0.5
                                if not(is_close(multiranger.front)):
                                    STATE = ADVANCE
                    elif STATE == GOAL:
                        if x < 3.7 and y < 1.5 and not line0:
                            vy = -VELOCITY
                        elif x < 3.7 and y > 1.5 and not line3:
                            vy = VELOCITY
                        if y < 0.2 or is_close(multiranger.right):
                            line0 = True
                        if y > 2.8 or is_close(multiranger.left):
                            line3 = True
                        if line3 and line0:
                            vx = VELOCITY
                            line0 = False
                            line3 = False
                        elif line0 and not is_close(multiranger.left):
                            vy = VELOCITY
                        elif line3 and not is_close(multiranger.right):
                            vy = -VELOCITY
                        if is_close(multiranger.down):
                             keep_flying = False
                                
                    motion_commander.start_linear_motion(vx, vy, 0)
                    x += vx * 0.1
                    y += vy * 0.1
                    if is_close(multiranger.up):
                        print("landing")
                        keep_flying = False
                    time.sleep(0.1)
                    print("{}  {}".format(round(x,2),round(y,2)))

