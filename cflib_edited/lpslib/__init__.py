# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
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
""" The LPS lib is an API that is used to communicate with Loco Positioning
anchors.

The main purpose of the lib is to manage LPP (Loco Positioning Protocol).

Initially it will use a Crazyflie with a Loco Positioning deck as a bridge to
transfer information to LoPo anchors, but may in the future use other means
of transportation.

Example:
cf = Crazyflie()
cf.open_link("radio://0/80/2M")

anchor = LoPoAnchor(cf)
anchor.set_position(1, (1.23, 4.56, 7.89))

"""
import cflib
from cflib.crazyflie import Crazyflie
from lpslib.lopoanchor import LoPoAnchor

'''uri = "radio://0/80/2M"'''

cf = Crazyflie()
cf.open_link("radio://0/80/2M")

anchor = LoPoAnchor(cf)
anchor.set_position(0, (0.00, 0.29, 0.15))
anchor.set_position(1, (2.74, 0.00, 1.08))
anchor.set_position(2, (0.00, 2.15, 1.08))
anchor.set_position(3, (3.64, 2.43, 0.15))
anchor.set_position(4, (4.68, 0.48, 2.54))
anchor.set_position(5, (1.86, 2.81, 2.54))


cf.close_link()

