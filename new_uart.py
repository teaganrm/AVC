import serial
import time

# constants

# servo commands
MIDDLE = b'z' # point wheels to middle position
LEFT23 = b'y'
LEFT45 = b'x'
LEFT68 = b'w'
LEFT90 = b'v'
RIGHT23 = b'u'
RIGHT45 = b't'
RIGHT68 = b's'
RIGHT90 = b'r'

# main drive motor
NEUTRAL = b'q'
FORWARD12 = b'p'
FORWARD15 = b'o'
FORWARD20 = b'n'
FORWARD30 = b'm'
FORWARD40 = b'l'
FORWARD50 = b'k'
FORWARD60 = b'j'
FORWARD70 = b'i'
FORWARD80 = b'h'
FORWARD90 =  b'g'
FORWARD100 = b'f'
BACKWARD12 = b'e'
BACKWARD15 = b'd'
BACKWARD20 = b'c'
BACKWARD30 = b'b'
BACKWARD40 = b'a'
BACKWARD50 = b'A'
BACKWARD60 = b'B'
BACKWARD70 = b'C'
BACKWARD80 = b'D'
BACKWARD90 =  b'E'
BACKWARD100 = b'F'

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

def program():
    '''
    based on how far bucket is away and what color bucket is use serial_port.write(CONSTANT)
    '''

serial_port.close()




