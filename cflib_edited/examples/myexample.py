import cflib.crtp
import cflib
from cflib.crazyflie import Crazyflie
from lpslib.lopoanchor import LoPoAnchor
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from threading import Thread
import logging
import random
import time

class MyExample:
	"""Example that connects to a Crazyflie and ramps the motors up/down and
	the disconnects"""

	def __init__(self, link_uri):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie(rw_cache='./cache')

		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)
			
		print('Connecting to %s' % link_uri)

	def _connected(self, link_uri):
		""" This callback is called form the Crazyflie API when a Crazyflie
		has been connected and the TOCs have been downloaded."""
		print('Connected')
		# Start a separate thread to do the motor test.
		# Do not hijack the calling thread!
		Thread(target=self._test_func).start()	
		
	def _connection_failed(self, link_uri, msg):
		"""Callback when connection initial connection fails (i.e no Crazyflie
		at the specified address)"""
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		"""Callback when disconnected after a connection has been made (i.e
		Crazyflie moves out of range)"""
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		"""Callback when the Crazyflie is disconnected (called in all cases)"""	
		print('Disconnected from %s' % link_uri)

	def _test_func(self):
		print('In the test function')
		thrust_mult = 1
		thrust_step = 500
		thrust = 26000
		pitch = 0
		roll = 0
		yawrate = 0

		# Unlock startup thrust protection
		self._cf.commander.send_setpoint(0, 0, 0, 0)

		while thrust >= 20000:
			self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
			time.sleep(0.1)
			if thrust >= 25000:
				thrust_mult = -1
			thrust += thrust_step * thrust_mult
		self._cf.commander.send_setpoint(0, 0, 0, 0)
			# Make sure that the last packet leaves before the link is closed
			# since the message queue is not flushed before closing
		time.sleep(0.1)
		self._cf.close_link()




if __name__ == '__main__':
	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	#  print('Scanning interfaces for Crazyflies...')
	#  available = cflib.crtp.scan_interfaces()
	#  print('Crazyflies found:')
	#  for i in available:
	#      print(i[0])

	# if len(available) > 0:
	#      le = MyExample('radio://0/80/2M/E7E7E7E701')
	#   else:
	#        print('No Crazyflies found, cannot run example')

	le = MyExample('radio://0/80/2M/E7E7E7E701')


