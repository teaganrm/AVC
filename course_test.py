import cv2
import keyboard
import numpy as np
import time
import serial
from realsense_depth import DepthCamera

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
FORWARD25 = b'H'
FORWARD30 = b'm'
FORWARD35 = b'I'
FORWARD40 = b'l'
FORWARD45 = b'J'
FORWARD50 = b'k'
FORWARD55 = b'K'
FORWARD60 = b'j'
FORWARD65 = b'L'
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

# window size
WINDOW_SIZE = 640
# vertical field of view of camera in degrees
FOV = 64
# number of pixels per degree
PIXELS_PER_DEGREE = round(WINDOW_SIZE / FOV)
# half of the window size
HALF_WINDOW_SIZE = round(WINDOW_SIZE / 2)
# distance threshold in mm
DISTANCE_THRESHOLD = 1000
RED_DISTANCE_THRESHOLD = 2000
# current speed
speed_var = 0
# message 
message = ''
# list of all speeds
all_speeds = [FORWARD12, FORWARD15, FORWARD20, FORWARD25, FORWARD30, FORWARD35, FORWARD40, FORWARD45, FORWARD50, FORWARD55, FORWARD60]
# indices to map to all_speeds
all_indices = [12, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
obstacleCount = 0

serial_port.write(NEUTRAL)
time.sleep(5)
serial_port.write(MIDDLE)
time.sleep(0.2)
serial_port.write(FORWARD20)
time.sleep(0.2)
serial_port.write(FORWARD30)


def show_distance(event, x, y, args, params):
    print(x, y)

def detect_largest_color(mask, color_name, color_frame):
    mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = None
    largest_contour_area = 0

    for mask_contour in mask_contours:
        contour_area = cv2.contourArea(mask_contour)
        if contour_area > largest_contour_area:
            largest_contour = mask_contour
            largest_contour_area = contour_area

    if largest_contour_area > 500:
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(color_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(color_frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return (x + w // 2, y + h // 2)
    else:
        return None

def detect_two_colors(mask, color_name, color_frame):
    mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = None
    largest_contour2 = None
    largest_contour_area = 0
    largest_contour2_area = 0

    for mask_contour in mask_contours:
        contour_area = cv2.contourArea(mask_contour)
        if contour_area > largest_contour_area:
            largest_contour2 = largest_contour
            largest_contour2_area = largest_contour_area
            largest_contour = mask_contour
            largest_contour_area = contour_area

    if largest_contour2_area > 500:
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(color_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(color_frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        x2, y2, w2, h2 = cv2.boundingRect(largest_contour2)
        cv2.rectangle(color_frame, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255), 2)
        cv2.putText(color_frame, color_name, (x2, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return (x + w // 2, y + h // 2, x2 + w2 // 2, y2 + h2 // 2)
    else:
        return None
    

def course_correct(center_x):
    # in this case, windowHalf = 320, pixelsPerDegree = 10
    global HALF_WINDOW_SIZE
    global PIXELS_PER_DEGREE
    # degrees to turn = (midpoint-half of window size) / pixels per degree. round to integer. 
    degreesToTurn = round((center_x - HALF_WINDOW_SIZE) / PIXELS_PER_DEGREE)
    # if degreesToTurn negative, turn left. else turn right
    direction = 'L' if (degreesToTurn < 0) else 'R'
    # we only care about magnitude once we know direction
    degreesToTurn = abs(degreesToTurn)
    # if we are within 5 degrees of being centered, consider the vehicle aligned
    # return value: aligned, direction, degreesToTurn
    if (degreesToTurn < 6):
        return (True, '', 0)
    else:
        return (False, direction, str(degreesToTurn))

def ramp_speed(targetSpeed):
    global speed_var
    global all_speeds
    global all_indices
    
    # only ramp if current speed < target speed
    if (speed_var >= targetSpeed):
        return
    currentIndex = 0
    targetIndex = 0
    # find current and target speeds and set indices
    for x in range(len(all_indices)):
        if (speed_var == all_indices[x]):
            currentIndex = x
        if (targetSpeed == all_indices[x]):
            targetIndex = x
    
    
	# only call UART commands within indices
    speeds = all_speeds[currentIndex, targetIndex+1]
    for s in speeds:
        serial_port.write(s)
        time.sleep(0.2)
    
	# set speed_var to target speed
    speed_var = targetSpeed
    
# each color function will print distance and message above center point
def blue_bucket_function(center_x, distance):
    global speed_var
    global message
    # always check to see if we're aligned
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        if aligned[1] == 'R':
            serial_port.write(RIGHT23) # turn right 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning right")
            serial_port.write(MIDDLE)

        elif aligned[1] == 'L':
            serial_port.write(LEFT23) # turn left 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning left")
            serial_port.write(MIDDLE)
        
    else:
        if (distance < 1500 and distance > 50): # if aligned and close, perform action
            startTime = time.time()
            serial_port.write(NEUTRAL)
            while time.time() - startTime < 2:
                print("stopping")
            serial_port.write(LEFT45)
            serial_port.write(FORWARD20)
            startTime = time.time()
            while time.time() - startTime < 0.2:
                print("forward 20")
            serial_port.write(FORWARD30)
            startTime = time.time()
            while time.time() - startTime < 4:
                print("go left for 4 seconds")
            serial_port.write(RIGHT68)
            startTime = time.time()
            while time.time() - startTime < 6:
                print("go right for 6 seconds")
            serial_port.write(MIDDLE)
            serial_port.write(FORWARD35)
            obstacleCount = obstacleCount +1
        else:
            message = " ALIGNED"    # if aligned but far
            #serial_port.write(MIDDLE) # realign servo
    cv2.putText(color_frame, "{}mm".format(distance) + message, (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)
    
def red_bucket_function(center_x, distance):
    global speed_var
    # always check to see if we're aligned
    #message = ""
    aligned = course_correct(center_x)
    if (not aligned[0]):
        #message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        if distance < 800 and distance > 50:
            serial_port.write(NEUTRAL)
            speed_var = 0 #sets speed variable to 0
            serial_port.close() # close serial port

        elif aligned[1] == 'R':
            serial_port.write(RIGHT23) # turn right 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning right")
            serial_port.write(MIDDLE)

        elif aligned[1] == 'L':
            serial_port.write(LEFT23) # turn left 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning left")
            serial_port.write(MIDDLE)
    else:
        if (distance < RED_DISTANCE_THRESHOLD): # if aligned and close, perform action
            #message = " PERFORM ACTION"
            serial_port.write(MIDDLE)
            if speed_var >= 30:
                serial_port.write(FORWARD30)
                speed_var = 30
            else:
                ramp_speed(30)
            startTime = time.time()
            while time.time() - startTime < 4: # go forward 60% for 4 seconds
                print("going through red buckets")
            obstacleCount = obstacleCount +1
        else:
            message = " ALIGNED"    # if aligned but far, probably do nothing
            serial_port.write(MIDDLE) # centralize servo
            ramp_speed(60) # go forward 60%

def wood_bucket_function(center_x, distance):
    global speed_var
    # always check to see if we're aligned
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        if aligned[1] == 'R':
            serial_port.write(RIGHT23) # turn right 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning right")
            serial_port.write(MIDDLE)

        elif aligned[1] == 'L':
            serial_port.write(LEFT23) # turn left 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning left")
            serial_port.write(MIDDLE)
    else:
        if (distance < 2000): # if aligned and close, perform action
            serial_port.write(MIDDLE)
            serial_port.write(FORWARD60)
            startTime = time.time()
            while time.time() - startTime < 4: # go forward 60% for 4 seconds
                print("going over ramp")
            serial_port.write(MIDDLE) # centralize servo
            obstacleCount = obstacleCount +1

        else:
            message = " ALIGNED"    # if aligned but far
            serial_port.write(MIDDLE) # centralize servo
            ramp_speed(60) # go forward 60%

# Capturing webcam footage
dc = DepthCamera()

cv2.namedWindow("Color frame")
# cv2.setMouseCallback("Color frame", show_distance)

# Create color ranges
lower_red1, upper_red1 = np.array([168, 100, 100]), np.array([180, 255, 255])  # Red color range
lower_red2, upper_red2 = np.array([0, 200, 200]), np.array([10, 255, 255])  # Red color range
lower_blue, upper_blue = np.array([100, 100, 0]), np.array([140, 255, 255])  # Blue color range
lower_yellow, upper_yellow = np.array([10, 100, 100]), np.array([40, 255, 255])  # Yellow color range
lower_wood, upper_wood = np.array([12, 50, 100]), np.array([40, 200, 200]) # Wood color range

lower_red_combined = np.minimum(lower_red1, lower_red2)
upper_red_combined = np.maximum(upper_red1, upper_red2)

# keyboard.on_press(on_key_event)

while True:
    ret, depth_frame, color_frame = dc.get_frame()  # Reading webcam footage
    img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)  # Converting BGR image to HSV format

    points = []

    red_mask = cv2.inRange(img, lower_red_combined, upper_red_combined)
    blue_mask = cv2.inRange(img, lower_blue, upper_blue)
    yellow_mask = cv2.inRange(img, lower_yellow, upper_yellow)
    wood_mask = cv2.inRange(img, lower_wood, upper_wood)

     # read only certain color depending on obstacle counter
    if obstacleCount in [0, 1, 3, 5]:
        blue_point = detect_largest_color(blue_mask, "Blue", color_frame)
        if (blue_point):
            points.append(blue_point)
        else:
            points.append((color_frame.shape[1] // 2, color_frame.shape[0] // 2))
    elif obstacleCount in [2]:
        wood_point = detect_largest_color(wood_mask, "Wood", color_frame)
        if (wood_point):
            points.append(wood_point)
        else:
            points.append((color_frame.shape[1] // 2, color_frame.shape[0] // 2))
    elif obstacleCount == 4:
        red_point = detect_two_colors(red_mask, "Red", color_frame)
        if (red_point):
            points.append(red_point)

    # for each point call its respective function
    for point in points:
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1], point[0]]
        if point == blue_point:
            blue_bucket_function(point[0], distance)
        elif point == wood_point:
            wood_bucket_function(point[0], distance)
        elif point == red_point:
            redMidpoint = (point[0] + point[2]) // 2
            red_bucket_function(redMidpoint, distance)
    cv2.imshow("Color frame", color_frame)  # Displaying webcam image

    cv2.waitKey(1)
