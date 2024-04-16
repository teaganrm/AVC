import cv2
import numpy as np
import serial
import time
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

# Capturing webcam footage
dc = cv2.VideoCapture(0)

cv2.namedWindow("Color frame")

while True:
    # ret, depth_frame, color_frame = dc.get_frame()  # Reading webcam footage
    ret, color_frame = dc.read()
    img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)  # Converting BGR image to HSV format

    points = []

    lower_red1, upper_red1 = np.array([168, 100, 100]), np.array([180, 255, 255])  # Red color range
    lower_red2, upper_red2 = np.array([0, 200, 200]), np.array([10, 255, 255])  # Red color range
    lower_blue, upper_blue = np.array([95, 70, 70]), np.array([130, 255, 255])  # Blue color range
    lower_yellow, upper_yellow = np.array([20, 65, 65]), np.array([40, 255, 255])  # Yellow color range

    lower_red_combined = np.minimum(lower_red1, lower_red2)
    upper_red_combined = np.maximum(upper_red1, upper_red2)

    red_mask = cv2.inRange(img, lower_red_combined, upper_red_combined)
    blue_mask = cv2.inRange(img, lower_blue, upper_blue)
    yellow_mask = cv2.inRange(img, lower_yellow, upper_yellow)

    red_point = detect_largest_color(red_mask, "Red", color_frame)
    blue_point = detect_largest_color(blue_mask, "Blue", color_frame)
    yellow_point = detect_largest_color(yellow_mask, "Yellow", color_frame)

    if red_point:
        points.append(red_point)
    elif blue_point:
        points.append(blue_point)
    elif yellow_point:
        points.append(yellow_point)
    else: 
        points.append((color_frame.shape[1] // 2, color_frame.shape[0] // 2))

    for point in points:
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        cv2.putText(color_frame, "point", (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0),
                    2)

    cv2.imshow("Color frame", color_frame)  # Displaying webcam image

    cv2.waitKey(1)
