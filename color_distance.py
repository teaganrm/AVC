import cv2
import numpy as np
from realsense_depth import DepthCamera

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
    
def course_correct(point, center_x):
    print("course correct")
    # check if the detected point is not in the center of the page with a threshold of 15 pixels
    while (point[0] <= center_x-15) or (point[0] >= center_x+15):
        #check if bucket is to the right or left of center and adjust
        if point[0] >= center_x:
            print("turn right") # put uart command here
        if point[0] <= center_x:
            print("turn left") # put uart command here
    
def blue_bucket_function(point, center_x, distance):
    print("blue test")
    course_correct(point, center_x)
    
    while distance > 1:
        print("go forward") # put uart command here

    print("stop") # put uart command here
    print("turn left") # put uart command here
    print("go forward 2 meters") # put uart command here
    print("turn right") # put uart command here
    print("go forward until next obstacle") # put uart command here


def yellow_bucket_function(point, center_x, distance):
    print("yellow test")
    course_correct(point, center_x)

    while distance > 1:
        print("go forward") # put uart command here

    print("stop") # put uart command here
    print("turn left") # put uart command here
    print("go forward 2 meters") # put uart command here
    print("turn right") # put uart command here
    print("go forward until next obstacle") # put uart command here

def red_bucket_function(point, center_x, distance):
    print("red test")
    course_correct(point, center_x)
    print("go forward") # put uart command here

    
# Capturing webcam footage
dc = DepthCamera()

cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()  # Reading webcam footage
    img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)  # Converting BGR image to HSV format

    # calculate center of image for course correction
    height, width, _ = img.shape
    center_x = width // 2

    points = []

    lower_red, upper_red = np.array([168, 120, 120]), np.array([180, 255, 255])  # Red color range
    lower_blue, upper_blue = np.array([100, 50, 50]), np.array([130, 255, 255])  # Blue color range
    lower_yellow, upper_yellow = np.array([10, 50, 50]), np.array([30, 255, 255])  # Yellow color range

    red_mask = cv2.inRange(img, lower_red, upper_red)
    blue_mask = cv2.inRange(img, lower_blue, upper_blue)
    yellow_mask = cv2.inRange(img, lower_yellow, upper_yellow)

    red_point = detect_largest_color(red_mask, "Red", color_frame)
    blue_point = detect_largest_color(blue_mask, "Blue", color_frame)
    yellow_point = detect_largest_color(yellow_mask, "Yellow", color_frame)

    if red_point:
        points.append(red_point)
    if blue_point:
        points.append(blue_point)
    if yellow_point:
         points.append(yellow_point)

    for point in points:
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1], point[0]]
        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)

    cv2.imshow("Color frame", color_frame)  # Displaying webcam image

    # check which color is being detected and call appropriate function
    for point in points:
        distance = depth_frame[point[1], point[0]]
        if point == blue_point:
            blue_bucket_function(point, center_x, distance)
        if point == yellow_point:
            yellow_bucket_function(point, center_x, distance)
        if point == red_point:
            red_bucket_function(point, center_x, distance)


    cv2.waitKey(1)


