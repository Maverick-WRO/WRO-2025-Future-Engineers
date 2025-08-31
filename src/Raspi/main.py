import cv2
import numpy as np
import serial
import serial.tools.list_ports
from ultralytics import YOLO
from picamera2 import Picamera2
import time

def find_arduino(baud=115200, timeout=1):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "ACM" in port.device or "USB" in port.device:
            print(f"[INFO] Found Arduino at {port.device}")
            return serial.Serial(port.device, baud, timeout=timeout)
    print("[WARN] No Arduino found, running in CAMERA-ONLY DEBUG mode")
    return None

ser = find_arduino()

picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

model = YOLO("best.pt")
class_map = {0: "green_block", 1: "red_block"}

BASE_SPEED = 60
TURN_ANGLE_LEFT = 60
TURN_ANGLE_RIGHT = 120
STRAIGHT_ANGLE = 90

def send_command(speed, angle):
    cmd = f"<SPD:{speed};ANG:{angle}>"
    if ser:
        ser.write(cmd.encode())
        print(f"[SEND] {cmd}")
    else:
        print(f"[DEBUG] {cmd}")

print("[INFO] WRO Autonomous Mode Started - Press Ctrl+C to stop")

try:
    while True:
        frame = picam2.capture_array()
        results = model(frame, imgsz=320, conf=0.6)

        detected = False
        action = "none"

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                label = class_map.get(cls_id, "unknown")

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = int((x1 + x2) / 2)

                if label == "red_block":
                    send_command(BASE_SPEED, TURN_ANGLE_LEFT)
                    action = "RED -> LEFT"
                    detected = True
                elif label == "green_block":
                    send_command(BASE_SPEED, TURN_ANGLE_RIGHT)
                    action = "GREEN -> RIGHT"
                    detected = True

        if not detected:
            send_command(BASE_SPEED, STRAIGHT_ANGLE)
            action = "STRAIGHT"

        annotated = results[0].plot()
        cv2.putText(annotated, f"Action: {action}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("WRO Detection", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\n[STOP] Manual interrupt")
finally:
    send_command(0, STRAIGHT_ANGLE)
    if ser:
        ser.close()
    cv2.destroyAllWindows()
    print("[INFO] Shutdown complete")
