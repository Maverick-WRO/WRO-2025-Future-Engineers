import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("best.pt")
cap = cv2.VideoCapture(0)

ccm = np.array([
    [0.9, -0.05, 0.0],
    [-0.05, 1.0, -0.05],
    [0.0, -0.05, 0.9]
])

def apply_color_correction(frame, matrix):
    corrected = frame.astype(np.float32)
    corrected = corrected @ matrix.T
    corrected = np.clip(corrected, 0, 255).astype(np.uint8)
    return corrected

while True:
    ret, frame = cap.read()
    if not ret: break

    corrected_frame = apply_color_correction(frame, ccm)
    results = model(corrected_frame)
    annotated = results[0].plot()

    cv2.imshow("Corrected Frame", corrected_frame)
    cv2.imshow("YOLO Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
