import cflib.crtp
import cflib
from cflib.crazyflie import Crazyflie
from lpslib.lopoanchor import LoPoAnchor
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import logging
import random
import time

cf = Crazyflie()

cf.open_link('radio://0/80/2M')

cf.close_link()
