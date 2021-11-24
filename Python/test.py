import struct # for values to bytes
import serial # to communicate with the hoverboard
from time import sleep # to wait


def command(steer, speed):
	'''
	Creates a bytearray for controlling the hoverboard

	:param steer: -1000...1000
	:param speed: -1000...1000
	:returns: command bytes
	'''
	startBytes = bytes.fromhex('ABCD')[::-1] # lower byte first
	steerBytes = struct.pack('h', steer)
	speedBytes = struct.pack('h', speed)
	# calculate checksum
	checksumBytes = bytes(a^b^c for (a, b, c) in zip(startBytes, steerBytes, speedBytes))

	cmd = startBytes+steerBytes+speedBytes+checksumBytes
	return cmd


uart = serial.Serial('COM3', 115200, timeout=1)

# setup test
SPEED_MAX_TEST = 300
iTestMax = SPEED_MAX_TEST
iTest = 0

s = int(input("Speed:"))

# ramp speed
while True:
	try:
		speed = SPEED_MAX_TEST-2*abs(iTest)
		
		cmd = command(0, s)
		print('\nSending:')
		print(f'speed: {speed}, cmd: {cmd}')
		uart.write(cmd)
		print('Receiving:')
		feedback = uart.read_all()
		#print(feedback)
		if feedback:
			cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed = struct.unpack('<hhhhhhH', feedback[2:16])
			print(f'cmd1: {cmd1}, cmd2: {cmd2}, speedR_meas: {speedR_meas}, speedL_meas: {speedL_meas}, batVoltage: {batVoltage}, boardTemp: {boardTemp}, cmdLed: {cmdLed}')

		# calculate next test speed
		iTest += 10
		if (iTest > iTestMax):
			iTest = -iTestMax
		
		sleep(0.1)
	except KeyboardInterrupt:
		break

uart.close()