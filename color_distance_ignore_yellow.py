import cv2
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
#distance threshold for red buckets 
RED_DISTANCE_THRESHOLD = 2000
# obstacle counter
obstacleCount = 0
# current speed
speed_var = 0
# list of all speeds
all_speeds = [FORWARD12, FORWARD15, FORWARD20, FORWARD25, FORWARD30, FORWARD35, FORWARD40, FORWARD45, FORWARD50, FORWARD55, FORWARD60]
# indices to map to all_speeds
all_indices = [12, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]

#call neutral to set commands 
serial_port.write(NEUTRAL) #DO NOT REMOVE
time.sleep(5) #DO NOT REMOVE

def show_distance(event, x, y, args, params):
    print(x, y)

def ramp_speed(targetSpeed):
    global speed_var
    global all_speeds
    global all_indices
    
    # only ramp if current speed < target speed
    if (speed_var >= targetSpeed):
        return
    currentIndex = 0;
    targetIndex = 0;
    # find current and target speeds and set indices
    for x in range(len(all_indices)):
        if (speed_var == all_indices[x]):
            currentIndex = x
        if (targetSpeed == all_indices[x]):
            targetIndex = x
    
    # only call UART commands within indices
    speeds = all_speeds[currentIndex:targetIndex+1]
    for s in speeds:
        serial_port.write(s)
        time.sleep(0.2)
    
    # set speed_var to target speed
    speed_var = targetSpeed

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
    if (degreesToTurn < 5):
        return (True, '', 0)
    else:
        return (False, direction, str(degreesToTurn))
    
# each color function will print distance and message above center point
def blue_bucket_function(center_x, distance):
    global speed_var
    # always check to see if we're aligned
    message = ""
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        # if not aligned but we are close to something, stop immediately
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
        if (distance < DISTANCE_THRESHOLD): # if aligned and close, perform action
            if speed_var >= 30:
                serial_port.write(FORWARD30)
                speed_var = 30
            else:
                ramp_speed(30)
            serial_port.write(LEFT45)
            startTime = time.time()
            while time.time() - startTime < 1: # go forward 30% for 1 seconds (45% left)
                print("going left around blue")
            
            serial_port.write(RIGHT23)
            startTime = time.time()
            while time.time() - startTime < 2: # go forward 30% for 2 seconds (23% right)
                print("going right around blue")

            obstacleCount += 1
            serial_port.write(MIDDLE)# realign servo in center position
            ramp_speed(60) # return to regular 60% speed
            if obstacleCount == 6: # check for final blue bucket 
                while time.time() - startTime < 4: # go forward 4 seconds before stop
                    print("going straight before stopping")
                serial_port.write(NEUTRAL) # stop vehicle
                print("stopped")
                speed_var = 0 #sets speed variable to 0
                serial_port.close() # close serial port
        else:
            message = " ALIGNED"    # if aligned but far
            serial_port.write(MIDDLE) # realign servo
            ramp_speed(60)
    cv2.putText(color_frame, "{}mm".format(distance) + message, (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)

def yellow_bucket_function(center_x, distance):
    global speed_var
    # always check to see if we're aligned
    message = ""
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
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
        if (distance < DISTANCE_THRESHOLD): # if aligned and close, perform action
            # ramp action
            serial_port.write(MIDDLE)
            ramp_speed(60)
            startTime = time.time()
            while time.time() - startTime < 4: # go forward 60% for 4 seconds
                print("going over ramp")

            obstacleCount += 1 
            serial_port.write(MIDDLE) # centralize servo
            ramp_speed(60) # go forward 60%

        else:
            message = " ALIGNED"    # if aligned but far
            serial_port.write(MIDDLE) # centralize servo
            ramp_speed(60) # go forward 60%
    cv2.putText(color_frame, "{}mm".format(distance) + message, (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)

def red_bucket_function(center_x, distance):
    global speed_var
    # always check to see if we're aligned
    message = ""
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
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
            message = " PERFORM ACTION"
            serial_port.write(MIDDLE)
            if speed_var >= 30:
                serial_port.write(FORWARD30)
                speed_var = 30
            else:
                ramp_speed(30)
            startTime = time.time()
            while time.time() - startTime < 4: # go forward 60% for 4 seconds
                print("going through red buckets")
        else:
            message = " ALIGNED"    # if aligned but far, probably do nothing
            serial_port.write(MIDDLE) # centralize servo
            ramp_speed(60) # go forward 60%
    cv2.putText(color_frame, "{}mm".format(distance) + message, (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)
    
# Capturing webcam footage
dc = DepthCamera()

cv2.namedWindow("Color frame")
# cv2.setMouseCallback("Color frame", show_distance)

# Create color ranges
lower_red, upper_red = np.array([168, 120, 120]), np.array([180, 255, 255])  # Red color range
lower_blue, upper_blue = np.array([100, 50, 50]), np.array([130, 255, 255])  # Blue color range
lower_yellow, upper_yellow = np.array([10, 50, 50]), np.array([30, 255, 255])  # Yellow color range

ramp_speed(30) # speed_var will be 30

# speed_var will be 0 when it completes all obstacles or
# when it stops because it got too close to an unaligned obstacle
while speed_var != 0:
    ret, depth_frame, color_frame = dc.get_frame()  # Reading webcam footage
    img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)  # Converting BGR image to HSV format

    points = []

    red_mask = cv2.inRange(img, lower_red, upper_red)
    blue_mask = cv2.inRange(img, lower_blue, upper_blue)
    yellow_mask = cv2.inRange(img, lower_yellow, upper_yellow)

    # read only certain color depending on obstacle counter
    if obstacleCount in [0, 1, 3, 5]:
        blue_point = detect_largest_color(blue_mask, "Blue", color_frame)
        if (blue_point):
            points.append(blue_point)
        else:
            points.append((color_frame.shape[1] // 2, color_frame.shape[0] // 2))
    elif obstacleCount in [2]:
        yellow_point = detect_largest_color(yellow_mask, "Yellow", color_frame)
        if (yellow_point):
            points.append(yellow_point)
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
        elif point == yellow_point:
            yellow_bucket_function(point[0], distance)
        elif point == red_point:
            redMidpoint = (point[0] + point[2]) // 2
            red_bucket_function(redMidpoint, distance)

    cv2.imshow("Color frame", color_frame)  # Displaying webcam image

    cv2.waitKey(1)
