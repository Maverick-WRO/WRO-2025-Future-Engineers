import cv2

def open_camera(index=0, width=640, height=480):
    cap = cv2.VideoCapture(index)
    cap.set(3, width)
    cap.set(4, height)
    return cap
