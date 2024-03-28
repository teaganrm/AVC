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

# Capturing webcam footage
dc = DepthCamera()

cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()  # Reading webcam footage
    img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)  # Converting BGR image to HSV format

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

    cv2.waitKey(1)
