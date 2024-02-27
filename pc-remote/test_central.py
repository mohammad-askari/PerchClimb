"""
An example showing how to write a simple program using the Nordic Semiconductor
(nRF) UART service.
"""
import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator
import pandas as pd
import numpy as np
from datetime import datetime

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

UART_SERVICE_UUID = "13012F00-F8C3-4F4A-A8F4-15CD926DA146"
UART_TX_CHAR_UUID = "13012F03-F8C3-4F4A-A8F4-15CD926DA452"
UART_RX_CHAR_UUID = "13012F03-F8C3-4F4A-A8F4-15CD926DA146"
UUID_Data = '13012F03-F8C3-4F4A-A8F4-15CD926DA152'

def handle_disconnect(_: BleakClient):
	print("Device disconnected, press enter to restart")
	# cancelling all tasks effectively ends the program
	for task in asyncio.all_tasks():
		task.cancel()
	

# def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
# 	dataint = int.from_bytes(data,'little')
# 	print(data)
# 	print("received:", dataint)
	
async def uart_terminal():
	"""This is a simple "terminal" program that uses the Nordic Semiconductor
	(nRF) UART service. It reads from stdin and sends each line of data to the
	remote device. Any data received from the device is printed to stdout.
	"""
	device = await BleakScanner.find_device_by_name('FAAV')

	if device is None:
		print("no matching device found")
		# await client.disconnect()
		for task in asyncio.all_tasks():
			task.cancel()
			return

	try:
		async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
			await client.start_notify(UUID_Data, data_notification_handler)

			print("Connected, start typing and press ENTER...")
			loop = asyncio.get_running_loop()

			while True:
				# This waits until you type a line and press ENTER.
				# A real terminal program might put stdin in raw mode so that things
				# like CTRL+C get passed to the remote device.
				data = await loop.run_in_executor(None, sys.stdin.buffer.readline)
				
				# data will be empty on EOF (e.g. CTRL+D on *nix)
				if not data:
					break

				datadecoded = data.decode().strip()
				# Extract numerical value from the input
				try:
					numerical_value = int(datadecoded[1:]) if datadecoded[1:] else 0

				except ValueError:
					print("Invalid numerical value")
				
				data_to_send = bytearray([ord(datadecoded[0])])
				data_to_send.extend(numerical_value.to_bytes(2, byteorder='big'))
				await client.write_gatt_char(UART_RX_CHAR_UUID, data_to_send)


	except Exception:
		print("exception while connecting/connected")

#################################### Data return ################################

class FAAV_Data:
	def __init__(self):
		self.is_first_datapoint = True
		self.recording_time = datetime.now().strftime("%Y.%m.%d.%H.%M")
		self.data_encoded_values = []
		self.df = pd.DataFrame
		self.data_dict = {'t': np.nan, 'i': np.nan, 'w': np.nan, 'v': np.nan, 'a': np.nan}
	
	def decode_and_store_data(self, data):
		# self.data_encoded_values.append(data)
		info_type, value = decode_info(data)
		self.data_dict[info_type] = value

		# If new "t" type is detected, append the current dictionary to DataFrame
		if info_type == 't':
			print(self.data_dict)
			df_latest = pd.DataFrame([self.data_dict], columns=['t','i','w','v', 'a'])
			# dt_string = datetime.now().strftime("%Y.%m.%d.%H.%bM")
			if self.is_first_datapoint:
				df_latest.to_csv('sensordata/FAAVdata_' + self.recording_time + '.csv', mode='a', header=True, index=False)
				self.is_first_datapoint = False
			else:
				df_latest.to_csv('sensordata/FAAVdata_' + self.recording_time + '.csv', mode='a', header=False, index=False)
			self.data_dict = {'v': np.nan, 'i': np.nan, 'w': np.nan, 't': np.nan}

		# end of data marker coming from robot
		if info_type == 'Z':
			self.recording_time = datetime.now().strftime("%Y.%m.%d.%H.%M")
			self.is_first_datapoint = True
	

Sensordata = FAAV_Data()


def data_notification_handler(sender, data: bytearray):
	data_int = int.from_bytes(data,'little')
	global Sensordata
	Sensordata.decode_and_store_data(data_int)

# Function to decode the encoded integer
def decode_info(encoded_value):
	# Define constants for the bit shifts and masks
	INFO_TYPE_SHIFT = 25
	INFO_TYPE_MASK = 0x7F << INFO_TYPE_SHIFT
	VALUE_MASK = 0x01FFFFFF
	# Extract information type
	info_type = (encoded_value & INFO_TYPE_MASK) >> INFO_TYPE_SHIFT
	
	# Extract value
	value = encoded_value & VALUE_MASK
	
	return chr(info_type), value

if __name__ == "__main__":
	while True:
		try:
			asyncio.run(uart_terminal())
		except asyncio.CancelledError:
			# task is cancelled on disconnect, so we ignore this error
			pass
