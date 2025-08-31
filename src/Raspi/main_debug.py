import cv2
from ultralytics import YOLO
from picamera2 import Picamera2

model = YOLO("best.pt")
picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

BASE_SPEED = 60
TURN_ANGLE_LEFT = 60
TURN_ANGLE_RIGHT = 120
STRAIGHT_ANGLE = 90
class_map = {0: "green_block", 1: "red_block"}

print("[DEBUG] Running without Arduino...")

while True:
    frame = picam2.capture_array()
    results = model(frame, imgsz=320, conf=0.6)

    action = "STRAIGHT"
    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = class_map.get(cls_id, "unknown")
            if label == "red_block":
                print(f"[CMD] SPD={BASE_SPEED}, ANG={TURN_ANGLE_LEFT}")
                action = "RED -> LEFT"
            elif label == "green_block":
                print(f"[CMD] SPD={BASE_SPEED}, ANG={TURN_ANGLE_RIGHT}")
                action = "GREEN -> RIGHT"

    annotated = results[0].plot()
    cv2.putText(annotated, f"Action: {action}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Debug Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
