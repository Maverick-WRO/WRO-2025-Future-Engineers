import cv2
import numpy as np

def detect_red_green(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    lower_green = np.array([40, 50, 50])
    upper_green = np.array([90, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    red_detected = cv2.countNonZero(mask_red) > 500
    green_detected = cv2.countNonZero(mask_green) > 500

    return red_detected, green_detected
