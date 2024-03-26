'''
demo python program that sends and recieves data via uart 
using the known command structure between EE and CS team 
'''

import serial
import time

# constants
START = b'z'  # clockwise turn at start command
STOP = b'y'  # stop command
SPEED = b'x'  # forward speed command
LEFT = b'w'  # turn left command
RIGHT = b'v'  # turn right command
BLANK = b'u'  # blank value
STARTMESSAGE = b't'  # denote start of message to EE board

'''
serial port should be initialized at the start
of course and closed at the end of course
'''

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, )

# allow port to initialize
time.sleep(1)

# function takes command as string, value as byte then sends data via uart
def send_command(command, value):
    if command == "START":
        tmp_command = START
    elif command == "STOP":
        tmp_command = STOP
    elif command == "SPEED":
        tmp_command = SPEED
    elif command == "LEFT":
        tmp_command = LEFT
    elif command == "RIGHT":
        tmp_command = RIGHT
    else:
        tmp_command = BLANK

    # checksum (xor)
    checksum = int.from_bytes(STARTMESSAGE, byteorder='little') ^ int.from_bytes(tmp_command, byteorder='little') ^ int.from_bytes(value, byteorder='little') 
    checksum = checksum.to_bytes(1, "big")

    # send data via uart
    serial_port.write(STARTMESSAGE)
    serial_port.write(tmp_command)
    serial_port.write(value)
    serial_port.write(checksum)   # checksum

send_command("LEFT", BLANK)

'''
while True:
    if serial_port.inWaiting()> 0:
        data = serial_port.read()
        print(data)
'''

# use serial_port.close() at end of course
