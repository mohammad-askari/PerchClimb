# -*- coding: utf-8 -*-

import asyncio

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover
from bleak import BleakError
import os
from datetime import datetime

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
import time


UUID_RXD = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
UUID_TXD = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

ble = BLERadio()

uart_connection = None

#################################### RUN FUNCTION ###################################

async def run():
	print('Looking for nRF58240 Peripheral Device ...')

	found = False
	uart_connection = None
	devices = await discover()
	robot = 'PerchClimb'

	for d in devices:       
		if robot == d.name:
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
							
		
							if input_str[0:8] == "transfer":
								try:
									if uart_connection and uart_connection.connected:
										uart_service = uart_connection[UARTService]
										uart_service.reset_input_buffer()
										uart_service.write(input_str.encode("utf-8"))
										print("Connected ...")
										line = uart_service.readline().decode("utf-8")
										numberOfPackets = 0
										if line[:4] == "meta":
											numberOfPackets = int(line[6:])
											print("Metadata received: ", numberOfPackets)

										if "-f" not in input_str:
											# save the sensor data only (excl. actuator commands) if "-full" flag is not present
											alltext = "Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg]\n"
											counter = 0
											experimental_data = []
											while uart_connection.connected:
												buffer = uart_service.read(60)
												if isinstance(buffer, bytearray):
													counter += 1
													for i in range(0, 6):
														data = convert_exp_data_to_str(buffer[i*10 : i*10+10])
														experimental_data.append(data)
														alltext += str(data.time) + ',' + str(data.current) + ',' + str(data.roll) + ',' + str(data.pitch) + ',' + str(data.yaw) + '\n'
													print("packets: ", counter)
													
													if counter == numberOfPackets:
														save_exp_data_as_csv(alltext)
														break
										
										
										else:
											# save full experimental data (incl. actuator commands) only if "-full" flag is used
											alltext = "Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg],Throttle [us],Aileron,Elevator,Rudder,Clutch,Body Hook,Tail Hook,Wing Open [pwm]\n"
											counter = 0
											experimental_data = []
											while uart_connection.connected:
												buffer = uart_service.read(60)
												if isinstance(buffer, bytearray):
													counter += 1
													for i in range(0, 3):
														data = convert_exp_data_to_str(buffer[i*20 : i*20+20])
														experimental_data.append(data)
														alltext += str(data.time) + ',' + str(data.current) + ',' + str(data.roll) + ',' + str(data.pitch) + ',' + str(data.yaw) + ',' + str(data.throttle) + ',' + str(data.aileron) + ',' + str(data.elevator) + ',' + str(data.rudder) + ',' + str(data.clutch) + ',' + str(data.body_hook) + ',' + str(data.tail_hook) + ',' + str(data.wing_open) + '\n'
													print("packets: ", counter)
													
													if counter == numberOfPackets:
														save_exp_data_as_csv(alltext)
														break
													
									await asyncio.sleep(0.1)
								except Exception as e:
									BLEconnected = False
									print(e)
									break

								

							else:
								try:
									if uart_connection and uart_connection.connected:
										uart_service = uart_connection[UARTService]
										while uart_connection.connected:
											print("serial input: ", input_str)
											uart_service.write(input_str.encode("utf-8"))
											
											# UNCOMMENT FOR CLIMBING AROUND
											if input_str[0:5] == "hover":
													time.sleep(2+10)
													commands = 'esc 1900, \n'
													uart_service.write(commands.encode("utf-8"))
											
											# uart_service.write(b'\n')
											# print(uart_service.readline().decode("utf-8"))
											break
								except Exception as e:
									pass
									
									
					except BleakError as e:
						print("Could not connect, retrying")
	if not found:
		print('Could not find ' + robot)


###################################### FUNCTIONS ##################################

class DataProcessor:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

def convert_exp_data_to_str(buffer):
	if len(buffer) == 10:
		data = {
			'time':      int.from_bytes(buffer[0:2],  byteorder='little', signed=False),
			'current':   int.from_bytes(buffer[2:4],  byteorder='little', signed=True),
			'roll':      int.from_bytes(buffer[4:6],  byteorder='little', signed=True),
			'pitch':     int.from_bytes(buffer[6:8],  byteorder='little', signed=True),
			'yaw':       int.from_bytes(buffer[8:10], byteorder='little', signed=True)
		}
		return DataProcessor(**data)
	
	elif len(buffer) == 20:
		data = { 
			'time':      int.from_bytes(buffer[0:2],   byteorder='little', signed=False),
			'current':   int.from_bytes(buffer[2:4],   byteorder='little', signed=True),
			'roll':      int.from_bytes(buffer[4:6],   byteorder='little', signed=True),
			'pitch':     int.from_bytes(buffer[6:8],   byteorder='little', signed=True),
			'yaw':       int.from_bytes(buffer[8:10],  byteorder='little', signed=True),
			'throttle':  int.from_bytes(buffer[10:12], byteorder='little', signed=True),
			'aileron':   int.from_bytes(buffer[12:13], byteorder='little', signed=True),
			'elevator':  int.from_bytes(buffer[13:14], byteorder='little', signed=True),
			'rudder':    int.from_bytes(buffer[14:15], byteorder='little', signed=True),
			'clutch':    int.from_bytes(buffer[15:16], byteorder='little', signed=True),
			'body_hook': int.from_bytes(buffer[16:17], byteorder='little', signed=True),
			'tail_hook': int.from_bytes(buffer[17:18], byteorder='little', signed=True),
			'wing_open': int.from_bytes(buffer[18:20], byteorder='little', signed=True)
		}
		return DataProcessor(**data)
	
	else:
		print("BAD DATA")
		print(len(buffer))


def save_exp_data_as_csv(alltext):
	print("Writing to file...")
	dt_string = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
	try:
		os.mkdir('sensordata')
	except Exception as e:
		pass
	h = open('sensordata/exp_' + dt_string + '.csv', "+w")
	h.write(alltext)
	h.close()	
	print("Data saved to file")