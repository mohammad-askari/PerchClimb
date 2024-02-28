# -*- coding: utf-8 -*-

import asyncio

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover
from bleak import BleakError
import re
from datetime import datetime
import csv 
# import time
# import pygame

UUID = 			'13012F01-F8C3-4F4A-A8F4-15CD926DA146'
UUID_time = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA147'
UUID_pitch = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA148'
UUID_roll = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA149'
UUID_yaw = 		'13012F01-F8C3-4F4A-A8F4-15CD926DA150'
UUID_current = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA151'


#################################### RUN FUNCTION ###################################

async def run():
	print('Looking for nRF58240 Peripheral Device...')

	found = False
	devices = await discover()

	for d in devices:       
		if 'Seeed XIAO nRF58240 BLE' == d.name:
			print('Found ', str(d.name))
			found = True

			BLEconnected = False

			while True:
				await asyncio.sleep(0.1)
				if not BLEconnected:
					try: 
						async with BleakClient(d.address) as client:
							print(f'Connected to {d.address}')

							# setup joystick
							# pygame.display.init()
							# pygame.joystick.init()
							# controller = pygame.joystick.Joystick(0)
							# controller.init()

							# val = int.from_bytes(await client.read_gatt_char(UUID), "big")
							# print("Command : ", val)
							# input_str = print("Options:\n l: LED enable\n t: timeout [s, 1]\n d: delay [s, 1] \n f: seq freqency [output Hz] \n c: current limit [~A] \n e: enable motor [Output Hz] \n s0: stop motor \n r0: run sequence \n v: elevator position [+/-90 deg] \n q: Multiply value by 2.5 \n w(82/71/66): Tune led Red/Green/Blue")
							
							ledCommand = ["w82", "w71", "w66"]
							iLed = 0

							# start = time.perf_counter()
							while True:
								# current_time = time.perf_counter()
								# elapsed_time = current_time-start
								# # print(elapsed_time)
								# start = current_time
								
								# controller/manual mode
								# pygame.event.pump()
								# if controller.get_button(0) > 0:
								# 	print("MANUAL MODE")
								# 	# input_str = input("Enter a command: ")
								# 	input_str = ledCommand[iLed % 3]
								# 	iLed +=1
								# 	# start = current_time
								# else:
								# 	# input_str = "m"+str(controller.get_axis(1)*1000)	
								# 	# print(input_str)
								# 	if controller.get_axis(0) < -0.5:
								# 		input_str = "w82"
								# 	elif controller.get_axis(0) > 0.5:
								# 		input_str = "w71"
								# 	# elif (abs(controller.get_axis(1) 
								# 	# .get_axis(1)*1000)
								# 	else:
								# 		input_str = "x0"

								# input_str = ledCommand[iLed % 3] # make the led alternate between R,G,B
								# iLed +=1

								input_str = input("Enter a command: ")

								# Use regular expression to extract letter and value from the input
								# print(input_str)
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
											try:
												# read characterisitics
												rtime = await client.read_gatt_char(UUID_time)
												print("step 01") # debugging aid
												rpitch = await client.read_gatt_char(UUID_pitch)
												print("step 02")
												rroll = await client.read_gatt_char(UUID_roll)
												print("step 03")
												ryaw = await client.read_gatt_char(UUID_yaw)
												print("step 04")
												rcurrent = await client.read_gatt_char(UUID_current)
												print("step 1")

												# write data to csv file
												write_csv(rtime.decode(), rpitch.decode(),rroll.decode(),ryaw.decode(),rcurrent.decode())
												print("step 2")

												# print(rtime.decode() )
												await asyncio.sleep(0.1)
											except Exception as e:
												BLEconnected = False
												break	

										try:
											await client.write_gatt_char(UUID, data)
											await asyncio.sleep(0.01)
										except Exception as e:
											BLEconnected = False
											break
									else:
										print("Invalid input. Value too large.")
								else:
									print("Invalid input. Please enter a letter followed by a value.")
								
								
								# await asyncio.sleep(0.1)
					except BleakError as e:
						print("Could not connect, retrying")
	if not found:
		print('Could not find Seeed XIAO nRF58240 BLE Peripheral')


###################################### FUNCTIONS ##################################

def write_csv(rtime,rpitch,rroll,ryaw,rcurrent):
	dt_string = datetime.now().strftime("%Y.%m.%d.%H.%M.%S")

	dtime = rtime.split(',')
	dpitch = rpitch.split(',')
	droll = rroll.split(',')
	dyaw = ryaw.split(',')
	dcurrent = rcurrent.split(',')

	with open('sensordata/climb'+ dt_string + '.csv',newline='') as csvfile:
		# fieldnames = ['Time [ms]','Yaw', 'Pitch', 'Roll', 'Current']
		writer = csv.writer(csvfile)
		writer.writerow(['Time [ms]','Yaw', 'Pitch', 'Roll', 'Current'])
		for i in range(len(dtime)):
			writer.writerow(dtime[i], dpitch[i], droll[i], dyaw[i], dcurrent[i])
		


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
