# -*- coding: utf-8 -*-

import asyncio

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover
from bleak import BleakError
import re, os
from datetime import datetime
import csv 
import pandas as pd

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
import time
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
UUID_throttle = '13012F01-F8C3-4F4A-A8F4-15CD926DA152'
UUID_aileron = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA153'
UUID_elevator = '13012F01-F8C3-4F4A-A8F4-15CD926DA154'
UUID_rudder = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA155'
UUID_wing_lock = '13012F01-F8C3-4F4A-A8F4-15CD926DA156'
UUID_body_hook = '13012F01-F8C3-4F4A-A8F4-15CD926DA157'
UUID_tail_hook = '13012F01-F8C3-4F4A-A8F4-15CD926DA158'
UUID_RXD     =  '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
UUID_TXD	 =  '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

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
							
		
							if input_str[0:8] == "transfer":
								try:
									if uart_connection and uart_connection.connected:
										uart_service = uart_connection[UARTService]
										uart_service.write(input_str.encode("utf-8"))
										print("Connected ...")
										uart_service = uart_connection[UARTService]
										line = uart_service.readline().decode("utf-8")
										numberOfPackets = 0
										if line[:4] == "meta":
											numberOfPackets = int(line[6:])
											print("Metadata received: ", numberOfPackets)


										alltext = "Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg],Throttle,Aileron,Elevator,Rudder,Wing Lock,Body Hook,Tail Hook\n"
										counter = 0
										experimental_data = []
										while uart_connection.connected:
											buffer = uart_service.read(60)
											if isinstance(buffer, bytearray):
												counter += 1
												# print("Buffer size is: {0}".format(len(buffer)))
												for i in range(0, 6):
													data = convert_exp_data_to_str(buffer[i*10 : i*10+10])
													experimental_data.append(data)
													alltext += str(data.time) + ',' + str(data.current) + ',' + str(data.roll) + ',' + str(data.pitch) + ',' + str(data.yaw) + ',' + str(data.throttle) + ',' + str(data.aileron) + ',' + str(data.elevator) + ',' + str(data.rudder) + ',' + str(data.wing_lock) + ',' + str(data.body_hook) + ',' + str(data.tail_hook) + '\n'
												print("packets: ", counter)
													# try:
													# 	print(data.time, data.current, data.roll, data.pitch, data.yaw)
													# except Exception as e:
													# 	pass
												
												if counter == numberOfPackets:
													print("Writing to file...")
													dt_string = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
													try:
														os.mkdir('sensordata')
													except Exception as e:
														pass
													h = open('sensordata/climb' + dt_string + '.csv', "+w")
													h.write(alltext)
													h.close()	
													print("Data saved to file")
													break
													# for i in range(0, 600):
													# 	print(experimental_data[i].time, experimental_data[i].current, experimental_data[i].roll, experimental_data[i].pitch, experimental_data[i].yaw)
													# 	counter = 0
													# ble.disconnect()
													# exit(0)
														
									
								
										

								# alltext = ""
								# numberofdata = 0
								# try:
								# 	if uart_connection and uart_connection.connected:
								# 		uart_service = uart_connection[UARTService]
								# 		while uart_connection.connected:
								# 			# uart_service.write(input_str.encode("utf-8"))
								# 			# uart_service.write(b'\n')
								# 			line = uart_service.readline().decode("utf-8")
								# 			numberofdata += 1
								# 			#print(line)
								# 			alltext += line 

								# 			if numberofdata == 1000:
								# 				break
								# 			# break
								# except Exception as e:
								# 	print(e)
								# 	BLEconnected = False
								# 	break

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
									#print("error:", e)
									# BLEconnected = False
									#break
									
							
							# await asyncio.sleep(0.1)
					except BleakError as e:
						print("Could not connect, retrying")
	if not found:
		print('Could not find PerchClimb')


###################################### FUNCTIONS ##################################

class DataProcessor:
	def __init__(self, time, current, roll, pitch, yaw, throttle, aileron, elevator, rudder, wing_lock, body_hook, tail_hook):
		self.time = time
		self.current = current
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.throttle = throttle
		self.aileron = aileron
		self.elevator = elevator
		self.rudder = rudder
		self.wing_lock = wing_lock
		self.body_hook = body_hook
		self.tail_hook = tail_hook

def convert_exp_data_to_str(buffer):
	if len(buffer) == 10:
		time = int.from_bytes(buffer[0:2], byteorder='little', signed=False)
		current = int.from_bytes(buffer[2:4], byteorder='little', signed=True)
		roll = int.from_bytes(buffer[4:6], byteorder='little', signed=True)
		pitch = int.from_bytes(buffer[6:8], byteorder='little', signed=True)
		yaw = int.from_bytes(buffer[8:10], byteorder='little', signed=True)
		throttle = int.from_bytes(buffer[10:12], byteorder='little', signed=True)
		aileron = int.from_bytes(buffer[12:14], byteorder='little', signed=True)
		elevator = int.from_bytes(buffer[14:16], byteorder='little', signed=True)
		rudder = int.from_bytes(buffer[16:18], byteorder='little', signed=True)
		wing_lock = int.from_bytes(buffer[18:20], byteorder='little', signed=True)
		body_hook = int.from_bytes(buffer[20:22], byteorder='little', signed=True)
		tail_hook = int.from_bytes(buffer[22:], byteorder='little', signed=True)
		

		return DataProcessor(time, current, roll, pitch, yaw, throttle, aileron, elevator, rudder, wing_lock, body_hook, tail_hook)
	else:
		print("BAD DATA")
		print(len(buffer))





def save_data_to_csv(self, filename):
	dt_string = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
	df = pd.DataFrame(self.data)
	df.to_csv(filename + dt_string + '.csv')

def write_csv(rtime,rpitch,rroll,ryaw,rcurrent,rthrottle,raileron,relevator,rrudder,rwing_lock,rbody_hook,rtail_hook):
	dt_string = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

	dtime = rtime.split(',')
	dpitch = rpitch.split(',')
	droll = rroll.split(',')
	dyaw = ryaw.split(',')
	dcurrent = rcurrent.split(',')
	dthrottle = rthrottle.split(',')
	daileron = raileron.split(',')
	delevator = relevator.split(',')
	drudder = rrudder.split(',')
	dwing_lock = rwing_lock.split(',')
	dbody_hook = rbody_hook.split()
	dtail_hook = rtail_hook.split()
	print("step a")
	dft = pd.DataFrame({'Time': dtime})
	dfr = pd.DataFrame({'Roll':droll})
	dfp = pd.DataFrame({'Pitch': dpitch})
	dfy = pd.DataFrame({'Yaw': dyaw})
	dfc = pd.DataFrame({'Current': dcurrent})
	dftr = pd.DataFrame({'Throttle': dthrottle})
	dfai = pd.DataFrame({'Aileron': daileron})
	dfel = pd.DataFrame({'Elevator': delevator})
	dfru = pd.DataFrame({'Rudder': drudder})
	dfwl = pd.DataFrame({'Wing Lock': dwing_lock})
	dfbh = pd.DataFrame({'Body Hook': dbody_hook})
	dfth = pd.DataFrame({'Tail Hook': dtail_hook})
	print("step b")
	df = pd.concat([dft, dfp, dfr, dfy, dfc, dftr, dfai, dfel, dfru, dfwl, dfbh, dfth], axis=1) # Concat tolerates different array lengths
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
