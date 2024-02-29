# -*- coding: utf-8 -*-

import asyncio

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover
from bleak import BleakError
import re
from datetime import datetime
import csv 
import pandas as pd

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
# import time
# import pygame

# import os
# print(os.listdir())
# print(os.getcwd())


UUID = 			'13012F01-F8C3-4F4A-A8F4-15CD926DA146'
UUID_time = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA147'
UUID_pitch = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA148'
UUID_roll = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA149'
UUID_yaw = 		'13012F01-F8C3-4F4A-A8F4-15CD926DA150'
UUID_current = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA151'

ble = BLERadio()

uart_connection = None

#################################### RUN FUNCTION ###################################

async def run():
	print('Looking for nRF58240 Peripheral Device...')

	found = False
	uart_connection = None
	devices = await discover()

	for d in devices:       
		if 'PerchClimb' == d.name:
			print('Found ', str(d.name))
			found = True

			BLEconnected = False

			while True:
				await asyncio.sleep(0.1)
				if not uart_connection:
					try: 
						for adv in ble.start_scan(ProvideServicesAdvertisement):
							if UARTService in adv.services:
								uart_connection = ble.connect(adv)
								print("Connected")
								break
						ble.stop_scan()
						while True:
							

							input_str = input("Enter a command: ")
							input_str += '\n'
							
		
							if input_str[0] == 'y':
								try:
									# rtime = await client.read_gatt_char(UUID_time)
									# print("step 01")
									# rpitch = await client.read_gatt_char(UUID_pitch)
									# print("step 02")
									# # try:
									# rroll = await client.read_gatt_char(UUID_roll)
									# # except Exception as e: 
									# # 	print(e)
									# print("step 03")
									# ryaw = await client.read_gatt_char(UUID_yaw)
									# print("step 04")
									# rcurrent = await client.read_gatt_char(UUID_current)
									# print("step 1")
									# write_csv(rtime.decode(), rpitch.decode(),rpitch.decode(),ryaw.decode(),rcurrent.decode())
									# print("step 2")

									# print(rtime.decode() )
									await asyncio.sleep(0.1)
								except Exception as e:
									BLEconnected = False
									break	

							try:
								if uart_connection and uart_connection.connected:
									uart_service = uart_connection[UARTService]
									while uart_connection.connected:
										uart_service.write(input_str.encode("utf-8"))
										uart_service.write(b'\n')
										print(uart_service.readline().decode("utf-8"))
										break
							except Exception as e:
								print(e)
								BLEconnected = False
								break
									
							
							# await asyncio.sleep(0.1)
					except BleakError as e:
						print("Could not connect, retrying")
	if not found:
		print('Could not find PerchClimb')


###################################### FUNCTIONS ##################################

def write_csv(rtime,rpitch,rroll,ryaw,rcurrent):
	dt_string = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

	dtime = rtime.split(',')
	dpitch = rpitch.split(',')
	droll = rroll.split(',')
	dyaw = ryaw.split(',')
	dcurrent = rcurrent.split(',')
	print("step a")
	dft = pd.DataFrame({'Time': dtime})
	dfp = pd.DataFrame({'Pitch': dpitch})
	dfr = pd.DataFrame({'Roll':droll})
	dfy = pd.DataFrame({'Yaw': dyaw})
	dfc = pd.DataFrame({'Current': dcurrent})
	print("step b")
	df = pd.concat([dft, dfp, dfr, dfy, dfc], axis=1) # Concat tolerates different array lengths
	df.drop(df.tail(1).index,inplace=True) # drop last row of END OF FRAME VALUE
	print("step c")
	try: 
		df.to_csv('sensordata/climb' + dt_string + '.csv')
	except Exception as e: 
		print(e)
	print("step end")
	# try:
	# 	with open(filename, newline='') as csvfile:
	# 		print("step c")
	# 		# fieldnames = ['Time [ms]','Yaw', 'Pitch', 'Roll', 'Current']
	# 		writer = csv.writer(csvfile)
	# 		writer.writerow(['Time [ms]','Yaw', 'Pitch', 'Roll', 'Current'])
	# 		print("step d")
	# 		for i in range(len(dtime)):
	# 			writer.writerow(dtime[i], dpitch[i], droll[i], dyaw[i], dcurrent[i])
	# 			print("step e")
	# except Exception as e: 
	# 	print(e)
			


######################################### MAIN ######################################
					
loop = asyncio.get_event_loop()
asyncio.ensure_future(run())
loop.run_forever()



# try:
# 	loop.run_until_complete(run())
# except KeyboardInterrupt:
# 	print('\nReceived Keyboard Interrupt')
# finally:
# 	print('Program finished')
