# —————————————————————————————— LIBRARY IMPORTS ————————————————————————————— #
import asyncio
from bleak import BleakClient, BleakScanner
from bleak import BleakGATTCharacteristic
from time import sleep
import os, threading
from datetime import datetime
import signal, sys
import communication


# ———————————————————————— TERMINAL ANSI CONTROL CODES ——————————————————————— #
class font:
   red 		  = "\033[91m"
   green 	  = "\033[92m"
   yellow 	  = "\033[93m"
   blue 	  = "\033[94m"
   purple 	  = "\033[95m"
   cyan 	  = "\033[96m"
   bold 	  = "\033[1m"
   underline  = "\033[4m"
   normal     = "\033[22m"
   reset 	  = "\033[0m"

class cursor:
   save 	  = "\033[s"
   restore 	  = "\033[u"
   clearright = "\033[K"
   clearleft  = "\033[1K"
   clearline  = "\033[2K"
   up 		  = "\033[A"
   down 	  = "\033[B"
   right 	  = "\033[C"
   left 	  = "\033[D"
 
# ————————————————————————————— GLOBAL VARIABLES ————————————————————————————— #
UUID_RXD  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UUID_TXD  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
bleName   = "PerchClimb"
promptStr = font.bold + font.blue + "> Enter Commands:\u00A0" + font.cyan

continueRunning = True
thereIsDataToSend = False
cliThreadHandle = None
dataToSend = bytearray()
packetCount = 0
fileContentPackets = []
fileContentType = communication.FILE_TYPE_SIMPLE
alltext = ""
connected = False
nextPercent = 0


# —————————————————————— TERMINAL CURSOR MOVER FUNCTIONS ————————————————————— #
def saveCursor():
	print(cursor.save, end="", flush=True)

def restoreCursor():
	print(cursor.restore + cursor.clearright + font.reset, end="", flush=True)

def showPrompt():
	print(promptStr, end="", flush=True)


# ————————————————————— DECODED DATA PROCESSING CALLBACK ————————————————————— #
async def dataReceiveCallback(_: BleakGATTCharacteristic, buffer: bytearray):
	global packetCount
	global fileContentPackets
	global fileContentType
	global alltext
	global nextPercent
	
	# Clear the line to overwrite the user input character if necessary
	restoreCursor()
	
	# Each element in the list can be a different decoded packet type
	decodedPackets = communication.decodeBytes(buffer)
	
	for packet in decodedPackets:		
		# String packet
		if isinstance(packet, communication.pktString_t):
			print(packet.str, end="")
		
		# Metadata packet
		elif isinstance(packet, communication.pktFileMetadata_t):
			packetCount = packet.packetCount
			fileContentType = packet.filetype
			nextPercent = 5
			
			fileContentPackets = [None] * (packetCount + 1)
			print(f"Metadata received. {packet.packetCount} packets of type {communication.getFileTypeName(packet.filetype)} will be received\n")
		
		# FileContent packet
		elif isinstance(packet, communication.pktFileContent_t):
			fileContentPackets[packet.packetNo] = packet
			if (float(packet.packetNo + 1) / packetCount) * 100 >= nextPercent:
				percentStr = "[%-20s] %d%%" % ('='*int(nextPercent/5), nextPercent)
				color = font.yellow if nextPercent < 100 else font.green
				print("\r" + cursor.up + color + percentStr + font.normal)
				nextPercent += 5
		
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
				print(font.bold + font.red + "Unknown file type received. Aborting..." + font.reset)
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
			foldername 	 = "expdata"
			filename     = "experiment_" + dateAsString + ".csv"
			fileaddress  = foldername + "/" + filename
			try:
				os.mkdir(foldername)
			except Exception as e:
				pass
			file = open(fileaddress, "+w")
			file.write(alltext)
			file.close()
			msg = "Data has been saved to the file: "
			print(font.bold + msg + font.green + fileaddress + font.reset)

	saveCursor()
	showPrompt()

# ——————————————————————— COMMAND LINE INTERFACE THREAD —————————————————————— #
# a thread to read input user commands and send them to the device via bluetooth
def cliThread():
	global continueRunning
	global thereIsDataToSend
	global dataToSend
	
	helpStr = font.bold + font.cyan + "help" + font.reset
	print("CLI thread started. Type " + helpStr + " for available commands.")
	
	try:
		while continueRunning:
			# wait until the entered command is sent
			# while thereIsDataToSend == True:
			# 	sleep(0.05)

			try:
				command = input(promptStr)
				if command != "":
					saveCursor()
					pktString = communication.pktString_t()
					pktString.str = command + "\n"
					pktString.strLen = len(pktString.str)

					commPacket = communication.commPacket_t()
					communication.createStringPacket(commPacket, pktString)
					sendBuffer = communication.convertCommPacketToByteArray(commPacket)
					
					thereIsDataToSend = True
					dataToSend = sendBuffer
			# ignore ctrl+d
			except EOFError:
				print()
				pass

	except Exception as e:
		pass


# ——————————————————————— BLUETOOTH CONNECTION FUNCTION —————————————————————— #
async def connectBluetooth():
	bleDevice = None
	
	print("Looking for nRF58240 peripheral device...")

	# scan until we find the device
	while True:
		# scan nearby bluetooth devices
		foundDevices = await BleakScanner.discover(timeout=2.0)
		for device in foundDevices:
			if device.name == bleName:
				bleDevice = device
				break
		
		bleNameStr = font.bold + bleName + font.normal
		if bleDevice == None:
			msg = "Could not find " + bleNameStr + ". Scanning again..."
			print(font.yellow + msg + font.reset)
		else:
			msg = "Found " + bleNameStr + ". Connecting..."
			print(font.yellow + msg + font.reset)
			return bleDevice


# ————————————————————————————————— MAIN LOOP ———————————————————————————————— #
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
		print(font.bold + font.red + "\n\nDisconnected" + font.reset)

	# the main loop that connects to device and starts interacting with the device
	while continueRunning:
		# connect to the device
		bleDevice = await connectBluetooth()
		connected = True
		bleClient = BleakClient(bleDevice, disconnected_callback=disconnectCallback)
		await bleClient.connect()
		print(font.bold + font.green + "Connected to the device\n" + font.reset)
		
		# show services of the device
		print("Available services: ", len(bleClient.services.services))
		for service in bleClient.services.services:
			print(bleClient.services.get_service(service))
		print()
		
		# set read callback function
		await bleClient.start_notify(UUID_TXD, dataReceiveCallback)
		
		# start the CLI thread
		if cliThreadHandle == None:
			cliThreadHandle = threading.Thread(target=cliThread, args=())
			cliThreadHandle.daemon = True
			cliThreadHandle.start()
		else:
			saveCursor()
			showPrompt()

		# device is ready, start interacting with the device
		while connected:
			# if there is command to send from the cliThread, send it
			if thereIsDataToSend == True:
				await bleClient.write_gatt_char(UUID_RXD, dataToSend)
				thereIsDataToSend = False
			
			await asyncio.sleep(0.2)


# ———————————————————————————— LOGGED DATA DECODER ——————————————————————————— #
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


# —————————————————————————————— SIGNAL CALLBACK ————————————————————————————— #
def signalHandler(sig, frame):
	global continueRunning
	global loop
	
	continueRunning = False
	loop.stop()
	msg = "Termination signal (CTRL+C) detected. Closing the client..."
	print("\r", end="")
	print(font.bold + font.red + msg + font.reset)
	sys.exit()


# —————————————————————————————— MAIN FUNCTION —————————————————————————————— #
# detects CTRL+C to kill the thread and ignore CTRL+Z
signal.signal(signal.SIGINT,  signalHandler)
if os.name != 'nt': 
	signal.signal(signal.SIGTSTP, signal.SIG_IGN)

# starts an infinite running loop, run() function, until terminated by the user
loop = asyncio.get_event_loop()
asyncio.ensure_future(run())
loop.run_forever()