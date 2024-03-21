import asyncio
import os
from datetime import datetime
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
import time
import threading
import communication

bluetoothMutex = threading.Lock()
#ioMutex = threading.Lock()

UUID = 			'13012F01-F8C3-4F4A-A8F4-15CD926DA146'
UUID_time = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA147'
UUID_pitch = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA148'
UUID_roll = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA149'
UUID_yaw = 		'13012F01-F8C3-4F4A-A8F4-15CD926DA150'
UUID_current = 	'13012F01-F8C3-4F4A-A8F4-15CD926DA151'
UUID_RXD     =  '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
UUID_TXD	 =  '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

#################################### RUN FUNCTION ###################################

async def run():
	print('Looking for nRF58240 Peripheral Device...')
	
	# scan bluetooth and connect
	uart_connection = connectToBLuetooth()
	
	if uart_connection == None:
		print("Couldn't find/connect-to the device. Exiting...")
		return
	elif uart_connection and uart_connection.connected:
		print("Connected")
	else:
		print("There was a problem connecting to the device. Exiting...")
		return

	uart_service = uart_connection[UARTService]

	#input("Enter anything to start file transfer: ")

	#uart_service = uart_connection[UARTService]

	#uart_service.write("transfer\n".encode("utf-8"))
	#print("Sent log request. Receiving...")

	# start the CLI thread
	cliThreadHandle = threading.Thread(target=cliThread, args=(uart_service))
	
	# main bluetooth read loop
	while True:
		packetCount = 0
		fileContentPackets = []
		MAX_NUMBER_OF_LOGS_IN_EACH_PACKET = 5
		LOG_DATA_LEN = 10
		
		bluetoothMutex.acquire()
		buffer = uart_service.read(64)
		bluetoothMutex.release()
		
		if isinstance(buffer, bytearray):
			# decodedPackets contains packets that decoded and each element in the list can be a different type
			decodedPackets = communication.decodeBytes(buffer)
			for packet in decodedPackets:
				
				# String packet
				if isinstance(packet, communication.pktString_t):
					print(packet.str)
				
				# Metadata packet
				elif isinstance(packet, communication.pktFileMetadata_t):
					packetCount = packet.packetCount
					fileContentPackets = [None] * packetCount
					print("Metadata received. {0} packets will be sent".format(packetCount))
				
				# FileContent packet
				elif isinstance(packet, communication.pktFileContent_t):
					fileContentPackets[packet.packetNo] = packet
				
				# FileSend packet (file send process is finished)
				elif isinstance(packet, communication.pktFileSend_t):
					print("File send process is finished. Checking missing packets...")
					# TODO: check missing parts

					# create CSV from the packets
					alltext = "Packet No, Time [ms], Current [adc], Roll [deg], Pitch [deg], Yaw [deg]\n"
					
					# convert all data to string
					for i in range(packetCount):
						if fileContentPackets[i] != None:
							for j in range(MAX_NUMBER_OF_LOGS_IN_EACH_PACKET):
								logData = convert_exp_data_to_str(fileContentPackets[i].data[j * LOG_DATA_LEN : j * LOG_DATA_LEN + LOG_DATA_LEN])
								alltext += str(i) + ',' +str(logData.time) + ',' + str(logData.current) + ',' + str(logData.roll) + ',' + str(logData.pitch) + ',' + str(logData.yaw) + '\n'
								
								# check if next loop has data. otherwise break the inner loop
								if j * LOG_DATA_LEN + LOG_DATA_LEN >= fileContentPackets[i].dataLen:
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
					print("Data saved to file: sensordata/climb/{0}.csv".format(filename))


def connectToBLuetooth():
	MAX_RETRY_COUNT = 20
	tries = 0
	ble = BLERadio()

	while tries < MAX_RETRY_COUNT:
		for adv in ble.start_scan(ProvideServicesAdvertisement):
			if UARTService in adv.services:
				return ble.connect(adv)
		
		time.sleep(0.5)
		tries += 1
	return None

# the thread that reads user input and sends the given command to the device via bluetooth
def cliThread(uart_service):
	while True:
		command = input()
		pktString = communication.pktString_t()
		pktString.str = command + "\n"
		pktString.strLen = len(pktString.str)

		commPacket = communication.commPacket_t()
		communication.createStringPacket(commPacket, pktString)
		sendBuffer = communication.convertCommPacketToByteArray(commPacket)
		
		bluetoothMutex.acquire()		
		uart_service.write(sendBuffer)
		bluetoothMutex.release()

###################################### FUNCTIONS ##################################

class LogData:
	def __init__(self, time, current, roll, pitch, yaw):
		self.time = time
		self.current = current
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw

def convert_exp_data_to_str(buffer):
	if len(buffer) == 10:
		time = int.from_bytes(buffer[0:2], byteorder='little', signed=False)
		current = int.from_bytes(buffer[2:4], byteorder='little', signed=True)
		roll = int.from_bytes(buffer[4:6], byteorder='little', signed=True)
		pitch = int.from_bytes(buffer[6:8], byteorder='little', signed=True)
		yaw = int.from_bytes(buffer[8:], byteorder='little', signed=True)
		return LogData(time, current, roll, pitch, yaw)
	else:
		print("Bad file segment: {0}".format(len(buffer)))




######################################### MAIN ######################################
					
loop = asyncio.get_event_loop()
asyncio.ensure_future(run())
loop.run_forever()

