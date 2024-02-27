# -*- coding: utf-8 -*-

import asyncio
from bleak import BleakClient
from bleak import discover
from bleak import BleakError
import re
from datetime import datetime
import pandas as pd

UUID = '13012F03-F8C3-4F4A-A8F4-15CD926DA146'
UUID_Voltage = '13012F03-F8C3-4F4A-A8F4-15CD926DA149'
UUID_Current = '13012F03-F8C3-4F4A-A8F4-15CD926DA150'
UUID_Water = '13012F03-F8C3-4F4A-A8F4-15CD926DA151'
UUID_Time = '13012F03-F8C3-4F4A-A8F4-15CD926DA152'

waiting_for_data = 0
#################################### RUN FUNCTION ###################################

async def run():
	print('Looking for nRF58240 Peripheral Device...')
	global waiting_for_data
	found = False
	devices = await discover()

	for d in devices:       
		if 'FAAV' == d.name:
			print('Found ', str(d.name))
			found = True

			while True:
				await asyncio.sleep(0.1)
				try: 
					async with BleakClient(d.address) as client:
						print(f'Connected to {d.address}')
						# val = int.from_bytes(await client.read_gatt_char(UUID), "big")
						# print("Command : ", val)
						# input_str = input("Options:\n l: LED enable\n t: timeout [s, 1]\n d: delay [s, 1] \n f: seq freqency [output Hz] \n c: current limit [~A] \n e: enable motor [Output Hz] \n s0: stop motor \n r0: run sequence \n v: elevator position [+/-90 deg] \nEnter command:  ")
						while True:
							if waiting_for_data == 0:
								input_str = input("Enter command:  ")

								# Use regular expression to extract letter and value from the input
								match = re.match(r'([A-Za-z])(\d+)', input_str)

								if match:
									letter, value = match.groups()
									value = int(value)
									if(value < 10000):
										# print("Command: ", letter, value)
										# Convert letter and value to bytes and write to GATT characteristic
										data = bytearray([ord(letter)])
										data.extend(value.to_bytes(2, byteorder='big'))

										if letter == 'z':
											await client.start_notify(UUID_Voltage, voltage_notification_handler)
											await client.start_notify(UUID_Current, current_notification_handler)
											await client.start_notify(UUID_Water, water_notification_handler)
											await client.start_notify(UUID_Time, time_notification_handler)
											waiting_for_data = 1
																						

										try:
											await client.write_gatt_char(UUID, data)
											await asyncio.sleep(0.1)
										except Exception as e:
											BLEconnected = False
											break
									else:
										print("Invalid input. Value too large.")

								else:
									print("Invalid input. Please enter a letter followed by a value.")
								# await asyncio.sleep(0.1)
							else:
								await asyncio.sleep(0.01)
				except BleakError as e:
					print("Could not connect, retrying")
			

	if not found:
		print('Could not find Seeed XIAO nRF58240 BLE Peripheral')


#################################### Data return ################################

class FAAV_Data:
	def __init__(self):
		self.voltage_values = []
		self.current_values = []
		self.water_values = []
		self.time_values = []

	def add_voltage_to_array(self, data):
		self.voltage_values.append(data)

	def add_current_to_array(self, data):
		self.current_values.append(data)

	def add_water_to_array(self, data):
		self.water_values.append(data)

	def add_time_to_array(self, data):
		self.time_values.append(data)
	
	def buffer_to_csv(self):
		# datetime object containing current date and time
		dt_string = datetime.now().strftime("%Y.%m.%d.%H.%M.%S")
		# dict = {'Voltage': self.voltage_values, 'Current': self.current_values} 
		dfv = pd.DataFrame({'Voltage': self.voltage_values})
		dfi = pd.DataFrame({'Current': self.current_values})
		dfw = pd.DataFrame({'Water': self.water_values})
		dft = pd.DataFrame({'Time': self.time_values})

		df = pd.concat([dft, dfv, dfi, dfw], axis=1) # Concat tolerates different array lengths
		df.drop(df.tail(1).index,inplace=True) # drop last row of END OF FRAME VALUE
		# Save to file
		df.to_csv('sensordata/FAAV_data_' + dt_string + '.csv')
		
	def clear_data(self):
		del self.voltage_values[:]
		del self.current_values[:]
		del self.water_values[:]
		del self.time_values[:]

Sensordata = FAAV_Data()

async def voltage_notification_handler(sender, data: bytearray):
	global Sensordata
	voltage = int.from_bytes(data,'little')
	Sensordata.add_voltage_to_array(voltage)
	# Handle end of file
	if(voltage == 1000000):
		await asyncio.sleep(0.5) # gives some time for transmission of other data to finish
		end_of_transmission_handler()
		return	

def current_notification_handler(sender, data: bytearray):
	data_int = int.from_bytes(data,'little')
	global Sensordata
	Sensordata.add_current_to_array(data_int)

def water_notification_handler(sender, data: bytearray):
	data_int = int.from_bytes(data,'little')
	global Sensordata
	Sensordata.add_water_to_array(data_int)

def time_notification_handler(sender, data: bytearray):
	data_int = int.from_bytes(data,'little')
	print(data_int)
	global Sensordata
	Sensordata.add_time_to_array(data_int)

def end_of_transmission_handler():
	global Sensordata
	global waiting_for_data
	Sensordata.buffer_to_csv()
	Sensordata.clear_data()
	waiting_for_data = 0
	print("Data transfer complete")

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
