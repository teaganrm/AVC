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
# obstacle counter
obstacleCount = 0

# keyboard event callback to increment obstacle counter
def on_key_event(event):
    global obstacleCount
    if (event.event_type == keyboard.KEY_DOWN):
        obstacleCount+=1
        if obstacleCount in [0, 2, 4, 6]:
            print("Looking for blues")
        elif obstacleCount in [1, 3]:
            print("Looking for yellows")
        elif obstacleCount == 5:
            print("Looking for reds")
        else:
            print("No more obstacles.")
        print("Count:", obstacleCount)


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
    if (degreesToTurn < 5):
        return (True, '', 0)
    else:
        return (False, direction, str(degreesToTurn))
    
# each color function will print distance and message above center point
def blue_bucket_function(center_x, distance):
    # always check to see if we're aligned
    message = ""
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        if aligned[1] == 'R':
            print("uart statement for turning right 23")
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning right")
            print("uart statement for centering servo")

        elif aligned[1] == 'L':
            print("uart statement for turning left 23")
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning left")
            print("uart statement for centering servo")
    else:
        if (distance < DISTANCE_THRESHOLD): # if aligned and close, perform action
            global obstacleCount
            print("uart statment for going forward 30%") # go forward 60%
            print("uart statment for turning left 45 degrees") # go forward 60%
            startTime = time.time()
            while time.time() - startTime < 0.5: # go forward 30% for 0.5 seconds (45% left)
                print("going left around blue")
            
            print("uart statment for turning right 23") # go forward 60%
            startTime = time.time()
            while time.time() - startTime < 2: # go forward 30% for 2 seconds (23% right)
                print("going right around blue")

            obstacleCount += 1
            print("uart statement for centering servo")# realign servo in center position
            print("uart statment for going forward 60%") # go forward 60%
            if obstacleCount == 7: # check for final blue bucket 
                while time.time() - startTime < 4: # go forward 4 seconds before stop
                    print("going right around blue")
                print("uart statment for stopping") # go forward 60%
                print("uart statment for diconnecting port") # go forward 60%
        else:
            message = " ALIGNED"    # if aligned but far
            print("uart statement for centering servo") # realign servo
            print("uart statment for going forward 60%") # go forward 60%
    cv2.putText(color_frame, "{}mm".format(distance) + message, (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)

def yellow_bucket_function(center_x, distance):
    # always check to see if we're aligned
    message = ""
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        if aligned[1] == 'R':
            print("uart statment for turning right 23") # turn right 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning right")
            print("uart statement for centering servo")

        elif aligned[1] == 'L':
            print("uart statment for turning left 23") 
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning left")
            print("uart statement for centering servo")
    else:
        if (distance < DISTANCE_THRESHOLD): # if aligned and close, perform action
            global obstacleCount
            if obstacleCount == 3: # ramp action
                print("uart statment for going forward 60%") # go forward 60%
                print("uart statement for centering servo")
                startTime = time.time()
                while time.time() - startTime < 4: # go forward 60% for 4 seconds
                    print("going over ramp")

            else:
                print("uart statment for going forward 30%") # go forward 30%
                print("uart statment for turning right 45 degrees") # turn right 45 degrees
                startTime = time.time()
                while time.time() - startTime < 0.5: # go forward 30% for 0.5 seconds (45% right)
                    print("going right around yellow bucket")
            
                print("uart statment for turning left 23") # go forward 60%
                startTime = time.time()
                while time.time() - startTime < 2: # go forward 30% for 2 seconds (23% left)
                    print("going left around yellow bucket")

            obstacleCount += 1 
            print("uart statement for centering servo")
            print("uart statment for going forward 60%") # go forward 60%

        else:
            message = " ALIGNED"    # if aligned but far
            print("uart statement for centering servo")
            print("uart statment for going forward 60%") # go forward 60%
    cv2.putText(color_frame, "{}mm".format(distance) + message, (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)

def red_bucket_function(center_x, distance):
    # always check to see if we're aligned
    aligned = course_correct(center_x)
    if (not aligned[0]):
        message = " " + aligned[1] + " " + aligned[2] # if not aligned, course correct
        if aligned[1] == 'R':
            print("uart statment for turning right 23") # turn right 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning right")
            print("uart statement for centering servo")

        elif aligned[1] == 'L':
            print("uart statment for turning left 23") # turn left 23 degrees to align
            startTime = time.time()
            while time.time() - startTime < 0.1: # recenter servo after 0.1 seconds
                print("aligning left")
            print("uart statement for centering servo")
    else:
        if (distance < RED_DISTANCE_THRESHOLD): # if aligned and close, perform action
            global obstacleCount
            message = " PERFORM ACTION"
            print("uart statement for centering servo")
            print("uart statment for going forward 60%")
            startTime = time.time()
            while time.time() - startTime < 4: # go forward 60% for 4 seconds
                print("going through red buckets")

            obstacleCount += 1 
            print("uart statement for centering servo")
            print("uart statment for going forward 60%") # go forward 60%
        else:
            message = " ALIGNED"    # if aligned but far, probably do nothing
            print("uart statement for centering servo") # centralize servo
            print("uart statment for going forward 60%") # go forward 60%
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

keyboard.on_press(on_key_event)

# in blue bucket function once it detects oc == 7 and 4 seconds pass
# increment oc to 8
while (obstacleCount < 8):
    ret, depth_frame, color_frame = dc.get_frame()  # Reading webcam footage
    img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)  # Converting BGR image to HSV format

    points = []

    red_mask = cv2.inRange(img, lower_red, upper_red)
    blue_mask = cv2.inRange(img, lower_blue, upper_blue)
    yellow_mask = cv2.inRange(img, lower_yellow, upper_yellow)

    # read only certain color depending on obstacle counter
    if obstacleCount in [0, 2, 4, 6]:
        blue_point = detect_largest_color(blue_mask, "Blue", color_frame)
        if (blue_point):
            points.append(blue_point)
    elif obstacleCount in [1, 3]:
        yellow_point = detect_largest_color(yellow_mask, "Yellow", color_frame)
        if (yellow_point):
            points.append(yellow_point)
    elif obstacleCount == 5:
        red_point = detect_two_colors(red_mask, "Red", color_frame)
        # only append red point if it found two red objects
        if (red_point):
            points.append(red_point)

    # for each point call its respective function
    for point in points:
        if point == blue_point:
            cv2.circle(color_frame, point, 4, (0, 0, 255))
            distance = depth_frame[point[1], point[0]]
            blue_bucket_function(point[0], distance)
        elif point == yellow_point:
            cv2.circle(color_frame, point, 4, (0, 0, 255))
            distance = depth_frame[point[1], point[0]]
            yellow_bucket_function(point[0], distance)
        else:
            # draw midpoint of both bounding boxes
            firstObject = tuple((point[0], point[1]))
            secondObject = tuple((point[2], point[3]))
            cv2.circle(color_frame, firstObject, 4, (0, 0, 255))
            cv2.circle(color_frame, secondObject, 4, (0, 0, 255))
            # only care about distance to 1 of them
            distance = depth_frame[point[1], point[0]]
            redMidpoint = (point[0] + point[2]) // 2
            red_bucket_function(redMidpoint, distance)

    cv2.imshow("Color frame", color_frame)  # Displaying webcam image

    cv2.waitKey(1)

print("Obstacle course complete.")
