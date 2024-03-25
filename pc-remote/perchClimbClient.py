import asyncio
from bleak import BleakClient, BleakScanner
from bleak import BleakError, BleakGATTCharacteristic
import os, time, threading
from datetime import datetime
import signal, sys
import communication
#---------------------------------------------------------------------------------------------------------------------
UUID_RXD = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
UUID_TXD = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
MAX_NUMBER_OF_LOGS_IN_EACH_PACKET = 5
LOG_DATA_LEN = 10
LOG_EX_DATA_LEN = 20

continueRunning = True
thereIsDataToSend = False
cliThreadHandle = None
dataToSend = bytearray()
packetCount = 0
fileContentPackets = []
fileContentType = communication.FILE_TYPE_SIMPLE
alltext = ""
#---------------------------------------------------------------------------------------------------------------------
# callback function when data is received
async def dataReceiveCallback(_: BleakGATTCharacteristic, buffer: bytearray):
	global packetCount
	global fileContentPackets
	global fileContentType
	global alltext
	
	# decodedPackets contains packets that has been decoded. each element in the list can be a different type
	decodedPackets = communication.decodeBytes(buffer)
	
	for packet in decodedPackets:		
		# String packet
		if isinstance(packet, communication.pktString_t):
			print(packet.str, end='')
		
		# Metadata packet
		elif isinstance(packet, communication.pktFileMetadata_t):
			packetCount = packet.packetCount
			fileContentType = packet.filetype
			
			fileContentPackets = [None] * (packetCount + 1)
			print("Metadata received. {0} packets of type {1} will be received".format(packet.packetCount, packet.filetype))
		
		# FileContent packet
		elif isinstance(packet, communication.pktFileContent_t):
			fileContentPackets[packet.packetNo] = packet
			print("{0}/{1}".format(packet.packetNo, packetCount))
		
		# FileSend packet (file send process is finished)
		elif isinstance(packet, communication.pktFileSend_t):
			print("File send process is finished. Checking missing packets...")
			# TODO: check missing parts
			
			# create CSV from the packets
			if fileContentType == communication.FILE_TYPE_SIMPLE:
				alltext = "Packet No,Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg]\n"
			elif fileContentType == communication.FILE_TYPE_EXTENDED:
				alltext = "Packet No,Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg],Throttle [us],Aileron,Elevator,Rudder,Clutch,Body Hook,Tail Hook,Wing Open [pwm]\n"
			else:
				print("Unknown file type received. Aborting...")
				return
			
			# convert all data to string
			for i in range(packetCount):
				if fileContentPackets[i] != None:
					for j in range(MAX_NUMBER_OF_LOGS_IN_EACH_PACKET):
						if fileContentPackets[i].filetype == communication.FILE_TYPE_SIMPLE:
							logData = decodeLogData(fileContentPackets[i].data[j * LOG_DATA_LEN : j * LOG_DATA_LEN + LOG_DATA_LEN])
							if logData != None:
								alltext += str(i) + ',' + str(logData.time) + ',' + str(logData.current) + ',' + str(logData.roll) + ',' + str(logData.pitch) + ',' + str(logData.yaw) + '\n'
						elif fileContentPackets[i].filetype == communication.FILE_TYPE_EXTENDED:
							logData = decodeLogDataEx(fileContentPackets[i].data[j * LOG_EX_DATA_LEN : j * LOG_EX_DATA_LEN + LOG_EX_DATA_LEN])
							if logData != None:
								alltext += str(i) + ',' + str(logData.time) + ',' + str(logData.current) + ',' + str(logData.roll) + ',' + str(logData.pitch) + ',' + str(logData.yaw) + ',' + str(logData.throttle) + ',' + str(logData.aileron) + ',' + str(logData.elevator) + ',' + str(logData.rudder) + ',' + str(logData.clutch) + ',' + str(logData.bodyHook) + ',' + str(logData.tailHook) + ',' + str(logData.wingOpen) + '\n'
						else:
							print("Unknown filetype {0} received. Aborting...".format(fileContentPackets[i].filetype))
							return

						
						# check if next loop has data. otherwise break the inner loop
						if fileContentPackets[i].filetype == communication.FILE_TYPE_SIMPLE:
							if j * LOG_DATA_LEN + LOG_DATA_LEN >= fileContentPackets[i].dataLen:
								break
						elif fileContentPackets[i].filetype == communication.FILE_TYPE_EXTENDED:
							if j * LOG_EX_DATA_LEN + LOG_EX_DATA_LEN >= fileContentPackets[i].dataLen:
								break
			
			# write to file
			filename = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
			try:
				os.mkdir('sensordata')
			except Exception as e:
				pass
			h = open('sensordata/climb' + filename + '.csv', "+w")
			h.write(alltext)
			h.close()	
			print("Data has been saved to the file: sensordata/climb/{0}.csv".format(filename))
#---------------------------------------------------------------------------------------------------------------------
# the thread that reads user input and sends the given command to the device via bluetooth
def cliThread():
	global continueRunning
	global thereIsDataToSend
	global dataToSend	

	print("CLI thread started. Enter your command:")
	
	try:
		while continueRunning:
			# wait until the entered command is sent
			while thereIsDataToSend == True:
				time.sleep(0.050)

			command = input(">")
			if command != "":
				pktString = communication.pktString_t()
				pktString.str = command + "\n"
				pktString.strLen = len(pktString.str)

				commPacket = communication.commPacket_t()
				communication.createStringPacket(commPacket, pktString)
				sendBuffer = communication.convertCommPacketToByteArray(commPacket)
				
				thereIsDataToSend = True
				dataToSend = sendBuffer
	except Exception as e:
		pass
#---------------------------------------------------------------------------------------------------------------------
# find and connect to the PerchClimb device
async def connectBluetooth():
	perchClimbDevice = None
	
	print('Looking for nRF58240 Peripheral Device...')

	# scan until we find the device
	while True:
		# scan nearby bluetooth devices
		foundDevices = await BleakScanner.discover(timeout=2.0)
		for device in foundDevices:
			if device.name == 'PerchClimb':
				perchClimbDevice = device
				break
		
		if perchClimbDevice == None:
			print("Could not find PerchClimb. Scanning again...")
		else:
			print('Found PerchClimb. Connecting...')
			return perchClimbDevice
#---------------------------------------------------------------------------------------------------------------------
# main loop
async def run():
	global continueRunning
	global cliThreadHandle
	global thereIsDataToSend
	global dataToSend
	connected = False
	
	# callback function on disconnection
	def disconnectCallback(_: BleakClient):
		print("Disconnected")
		connected = False

	# the main loop that connects to device and starts interacting with the device
	while continueRunning:
		# connect to the device
		perchClimbDevice = await connectBluetooth()
		connected = True

		bleClient = BleakClient(perchClimbDevice, disconnected_callback=disconnectCallback)
		await bleClient.connect()
		print("Connected to the device\n")
		
		# show services of the device
		print("Available services: ", len(bleClient.services.services))
		for service in bleClient.services.services:
			print(bleClient.services.get_service(service))
		print("\n")
		
		# set read callback function
		await bleClient.start_notify(UUID_TXD, dataReceiveCallback)
		
		# start the CLI thread
		cliThreadHandle = threading.Thread(target=cliThread, args=())
		cliThreadHandle.start()

		# device is ready, start interacting with the device
		while connected:
			# if there is command to send from the cliThread, send it
			if thereIsDataToSend == True:
				await bleClient.write_gatt_char(UUID_RXD, dataToSend)
				thereIsDataToSend = False
			
			await asyncio.sleep(0.2)
#---------------------------------------------------------------------------------------------------------------------			
class LogData:
	def __init__(self, time, current, roll, pitch, yaw):
		self.time = time
		self.current = current
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw

class LogDataEx:
	def __init__(self, time, current, roll, pitch, yaw, throttle, aileron, elevator, rudder, clutch, bodyHook, tailHook, wingOpen):
		self.time = time
		self.current = current
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.throttle = throttle
		self.aileron = aileron
		self.elevator = elevator
		self.rudder = rudder
		self.clutch = clutch
		self.bodyHook = bodyHook
		self.tailHook = tailHook
		self.wingOpen = wingOpen

def decodeLogData(buffer: bytearray):
	if len(buffer) == 10:
		time    = int.from_bytes(buffer[0:2], byteorder='little', signed=False)
		current = int.from_bytes(buffer[2:4], byteorder='little', signed=True)
		roll    = int.from_bytes(buffer[4:6], byteorder='little', signed=True)
		pitch   = int.from_bytes(buffer[6:8], byteorder='little', signed=True)
		yaw     = int.from_bytes(buffer[8:], byteorder='little', signed=True)

		return LogData(time, current, roll, pitch, yaw)
	else:
		print("Bad data in logs: ", len(buffer))

def decodeLogDataEx(buffer: bytearray):
	if len(buffer) == 20:
		time     = int.from_bytes(buffer[0:2], byteorder='little', signed=False)
		current  = int.from_bytes(buffer[2:4], byteorder='little', signed=True)
		roll     = int.from_bytes(buffer[4:6], byteorder='little', signed=True)
		pitch    = int.from_bytes(buffer[6:8], byteorder='little', signed=True)
		yaw      = int.from_bytes(buffer[8:], byteorder='little', signed=True)
		throttle = int.from_bytes(buffer[10:12], byteorder='little', signed=True)
		aileron  =  int.from_bytes(buffer[12:13], byteorder='little', signed=True)
		elevator = int.from_bytes(buffer[13:14], byteorder='little', signed=True)
		rudder   = int.from_bytes(buffer[14:15], byteorder='little', signed=True)
		clutch   = int.from_bytes(buffer[15:16], byteorder='little', signed=True)
		bodyHook = int.from_bytes(buffer[16:17], byteorder='little', signed=True)
		tailHook = int.from_bytes(buffer[17:18], byteorder='little', signed=True)
		wingOpen = int.from_bytes(buffer[18:20], byteorder='little', signed=True)

		return LogDataEx(time, current, roll, pitch, yaw, throttle, aileron, elevator, rudder, clutch, bodyHook, tailHook, wingOpen)
	else:
		print("Bad data in logs: ", len(buffer))
#---------------------------------------------------------------------------------------------------------------------			
def signal_handler(sig, frame):
	global continueRunning
	global loop
	
	continueRunning = False
	loop.stop()
	sys.exit(0)
#---------------------------------------------------------------------------------------------------------------------
######################################### MAIN ######################################
# detect ctrl+c to kill the thread
signal.signal(signal.SIGINT, signal_handler)

loop = asyncio.get_event_loop()
asyncio.ensure_future(run())
loop.run_forever()


