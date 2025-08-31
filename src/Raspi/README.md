# WRO2025 Future Engineers - Self Driving Robot

This repository contains the complete **Raspberry Pi + Arduino software stack** for our WRO 2025 Future Engineers project.  
The robot uses a Raspberry Pi 4 (with PiCamera2 + YOLOv8 for vision) and an Arduino UNO (for motor + servo + sensors).

---

## 📂 Project Structure

```
WRO2025-Robot/
│── README.md
│── requirements.txt
│
├── vision/
│   ├── detectColors.py        # Basic HSV red/green detection
│   ├── util.py                # Helper functions
│   ├── camera.py              # Simple camera interface
│
├── controllers/
│   ├── ackermann_controller.py # Ackermann steering model
│   ├── hybrid_controller.py    # Hybrid driving logic
│   ├── follow_gap.py           # Follow-the-gap (LIDAR-like logic)
│
├── main/
│   ├── camtest.py             # ✅ Final integrated YOLO + Arduino control
│   ├── camtest_debug.py       # Debug mode (no Arduino required)
│   ├── yolo_inference.py      # Standalone YOLO detection viewer
│   ├── color_correction.py    # Experimental IR bleeding correction
```

---

## ⚡ Requirements

On **Raspberry Pi**:

```bash
sudo apt update && sudo apt upgrade -y
pip install opencv-python numpy pyserial ultralytics picamera2
```

---

## ▶️ Usage

### 1. Run YOLO object detection (debug only)
```bash
python3 main/yolo_inference.py
```

### 2. Test color correction
```bash
python3 main/color_correction.py
```

### 3. Debug pipeline (no Arduino)
```bash
python3 main/camtest_debug.py
```

### 4. Full autonomy (with Arduino + motors)
```bash
python3 main/camtest.py
```

---

## 🔧 Arduino Side

The Arduino listens for serial commands from the Pi in the format:

```
<SPD:X;ANG:Y>
```

- `SPD` = speed (0–100 percent, scaled to PWM 120–255)  
- `ANG` = servo angle (0–180)  

Example commands:
```
<SPD:60;ANG:90>   # forward straight
<SPD:60;ANG:60>   # forward left
<SPD:60;ANG:120>  # forward right
<SPD:0;ANG:90>    # stop
```

---

## 🏁 WRO Challenge Logic

- **Red pillar** → robot steers LEFT  
- **Green pillar** → robot steers RIGHT  
- **No detection** → robot continues STRAIGHT  
- **Obstacle (ultrasonic)** → STOP (safety)

