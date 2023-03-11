import struct # for values to bytes
import serial # to communicate with the hoverboard
import keyboard
from time import sleep # to wait


def command(steer, speed, brake, driveMode):
	'''
	Creates a bytearray for controlling the hoverboard

	:param steer: -1000...1000
	:param speed: -1000...1000
	:param brake: 0...1000
	:param driveMode: SPD_MODE=2, TRQ_MODE=3
	:returns: command bytes
	'''
	startBytes = bytes.fromhex('ABCD')[::-1] # lower byte first
	steerBytes = struct.pack('h', steer)
	speedBytes = struct.pack('h', speed)
	brakeBytes = struct.pack('h', brake)
	driveModeBytes = struct.pack('h', driveMode)
	# calculate checksum
	checksumBytes = bytes(a^b^c^d^e for (a, b, c, d, e) in zip(startBytes, steerBytes, speedBytes, brakeBytes, driveModeBytes))

	cmd = startBytes+steerBytes+speedBytes+brakeBytes+driveModeBytes+checksumBytes
	return cmd


uart = serial.Serial('COM4', 115200, timeout=1)

# setup test
SPEED_MAX_TEST = 300
iTestMax = SPEED_MAX_TEST
iTest = 0


s = int(input("Speed:"))

# ramp speed
while True:
	try:
		if keyboard.is_pressed("0"):
			s = int(input("Speed:"))
		
		cmd = command(0, s, 0, 3) #no brake, torque mode
		#print(f'\nSending: speed: {s}, cmd: {cmd}')
		uart.write(cmd)
		#print('Receiving:')
		feedback = uart.read_all()
		#print(feedback)
		if feedback:
			try:
				cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed = struct.unpack('<hhhhhhH', feedback[2:16])
				#print(f'\nRecieved: cmd1: {cmd1}, cmd2: {cmd2}, speedR_meas: {speedR_meas}, speedL_meas: {speedL_meas}, batVoltage: {batVoltage}, boardTemp: {boardTemp}, cmdLed: {cmdLed}')
				print(f'\nRecieved: speedR_meas: {speedR_meas}, batVoltage: {batVoltage}')
			except:
					print("read data error")

		sleep(0.1)
	except KeyboardInterrupt:
		break

uart.close()