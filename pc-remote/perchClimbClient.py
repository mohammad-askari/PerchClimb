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
bleName  = "PerchClimb"

continueRunning = True
thereIsDataToSend = False
cliThreadHandle = None
dataToSend = bytearray()
packetCount = 0
fileContentPackets = []
fileContentType = communication.FILE_TYPE_SIMPLE
alltext = ""
connected = False
nextPerncent = 0
#---------------------------------------------------------------------------------------------------------------------
# callback function when data is received
async def dataReceiveCallback(_: BleakGATTCharacteristic, buffer: bytearray):
	global packetCount
	global fileContentPackets
	global fileContentType
	global alltext
	global nextPerncent
	
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
			nextPerncent = 10
			
			fileContentPackets = [None] * (packetCount + 1)
			print(f"Metadata received. {packet.packetCount} packets of type {communication.getFileTypeName(packet.filetype)} will be received")
		
		# FileContent packet
		elif isinstance(packet, communication.pktFileContent_t):
			fileContentPackets[packet.packetNo] = packet
			if (float(packet.packetNo + 1) / packetCount) * 100 >= nextPerncent:
				print(f"{nextPerncent}%   ", end='')
				nextPerncent += 10
				if nextPerncent == 110:
					print("\n")
			#print(f"{packet.packetNo + 1:>{len(str(packetCount))}} / {packetCount}")
		
		# FileSend packet (file send process is finished)
		elif isinstance(packet, communication.pktFileSend_t):
			print("File send process is finished. Checking missing packets...")
			# TODO: check missing parts
			
			# define mappings
			csvHeader = {
				communication.FILE_TYPE_SIMPLE:   "Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg]\n",
				communication.FILE_TYPE_EXTENDED: "Time [ms],Current [adc],Roll [deg],Pitch [deg],Yaw [deg],Throttle [µs],Aileron,Elevator,Rudder,Clutch,Body Hook,Tail Hook,Wing Open [pwm]\n"
			}
			logLength = {
				communication.FILE_TYPE_SIMPLE:   communication.LOG_SIMPLE_LEN,
				communication.FILE_TYPE_EXTENDED: communication.LOG_EXTENDED_LEN
			}

			# create CSV from the packets
			try:
				alltext = csvHeader[fileContentType]
				MAX_NUMBER_OF_LOGS_IN_EACH_PACKET = int(communication.MAX_FILECONTENT_DATALEN / logLength[fileContentType])
			except KeyError:
				print("Unknown file type received. Aborting...")
				return

			print(f"Received {packetCount} packets, each containing {MAX_NUMBER_OF_LOGS_IN_EACH_PACKET} logs at max.")
			print("Creating CSV file from received packets...")
			
			# convert all data to string
			for i in range(packetCount):
				if fileContentPackets[i] != None:
					for j in range(MAX_NUMBER_OF_LOGS_IN_EACH_PACKET):
						logData = decodeLogData(fileContentPackets[i].data[j * logLength[fileContentType] : (j+1) * logLength[fileContentType]])
						if logData != None:
							alltext += ','.join(map(str, logData.values())) + '\n'

						# check if next loop has data. otherwise, break the inner loop
						if (j+1) * logLength[fileContentType] >= fileContentPackets[i].dataLen:
							break
			
			# write to file
			dateAsString = datetime.now().strftime("%Y-%m-%d %H-%M-%S")
			foldername = "expdata"
			filename = "experiment_" + dateAsString + ".csv"
			try:
				os.mkdir(foldername)
			except Exception as e:
				pass
			h = open(foldername + "/" + filename, "+w")
			h.write(alltext)
			h.close()	
			print("Data has been saved to the file: " + foldername + "/" + filename)
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

			command = input("> ")
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
# find and connect to the specified device name
async def connectBluetooth():
	bleDevice = None
	
	print("Looking for nRF58240 Peripheral Device...")

	# scan until we find the device
	while True:
		# scan nearby bluetooth devices
		foundDevices = await BleakScanner.discover(timeout=2.0)
		for device in foundDevices:
			if device.name == bleName:
				bleDevice = device
				break
		
		if bleDevice == None:
			print("Could not find " + bleName + ". Scanning again...")
		else:
			print("Found " + bleName + ". Connecting...")
			return bleDevice
#---------------------------------------------------------------------------------------------------------------------
# main loop
async def run():
	global continueRunning
	global cliThreadHandle
	global thereIsDataToSend
	global dataToSend
	global connected
	
	# callback function on disconnection
	def disconnectCallback(_: BleakClient):
		global connected
		connected = False
		print("Disconnected")

	# the main loop that connects to device and starts interacting with the device
	while continueRunning:
		# connect to the device
		bleDevice = await connectBluetooth()
		connected = True

		bleClient = BleakClient(bleDevice, disconnected_callback=disconnectCallback)
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
		if cliThreadHandle == None:
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
def decodeLogData(buffer: bytearray):
	if len(buffer) == communication.LOG_SIMPLE_LEN:
		data = {
			'time':     int.from_bytes(buffer[0:2],  byteorder='little', signed=False),
			'current':  int.from_bytes(buffer[2:4],  byteorder='little', signed=True),
			'roll':     int.from_bytes(buffer[4:6],  byteorder='little', signed=True),
			'pitch':    int.from_bytes(buffer[6:8],  byteorder='little', signed=True),
			'yaw':      int.from_bytes(buffer[8:10], byteorder='little', signed=True)
		}
		return data

	elif len(buffer) == communication.LOG_EXTENDED_LEN:
		data = { 
			'time':     int.from_bytes(buffer[0:2],   byteorder='little', signed=False),
			'current':  int.from_bytes(buffer[2:4],   byteorder='little', signed=True),
			'roll':     int.from_bytes(buffer[4:6],   byteorder='little', signed=True),
			'pitch':    int.from_bytes(buffer[6:8],   byteorder='little', signed=True),
			'yaw':      int.from_bytes(buffer[8:10],  byteorder='little', signed=True),
			'throttle': int.from_bytes(buffer[10:12], byteorder='little', signed=True),
			'aileron':  int.from_bytes(buffer[12:13], byteorder='little', signed=True),
			'elevator': int.from_bytes(buffer[13:14], byteorder='little', signed=True),
			'rudder':   int.from_bytes(buffer[14:15], byteorder='little', signed=True),
			'clutch':   int.from_bytes(buffer[15:16], byteorder='little', signed=True),
			'bodyHook': int.from_bytes(buffer[16:17], byteorder='little', signed=True),
			'tailHook': int.from_bytes(buffer[17:18], byteorder='little', signed=True),
			'wingOpen': int.from_bytes(buffer[18:20], byteorder='little', signed=True)
		}
		return data
	
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