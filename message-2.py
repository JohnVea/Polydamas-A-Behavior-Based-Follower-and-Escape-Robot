import torch
import cv2
from picamera2 import Picamera2
import numpy as np
import atexit
import serial
import time
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

# Load YOLOv5
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', trust_repo=True)
model.eval()

# Serial setup
serial_port = serial.Serial(
    port='/dev/ttyUSB0',  # Update with your serial port
    baudrate=9600,
    timeout=1
)

# Stereo camera initialization
left_cam = Picamera2(0)
right_cam = Picamera2(1)
for cam in [left_cam, right_cam]:
    cam.preview_configuration.main.size = (640, 480)
    cam.preview_configuration.main.format = "RGB888"
    cam.configure("preview")
    cam.start()

# StereoBM and calibration
stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)
focal_length_px = 1029  # Use calibrated focal length
baseline_m = 0.05       # Distance between stereo cameras in meters

def send_motor_output(forward, right):
    if forward > 0:
        command = "FORWARD\n"
    elif forward < 0:
        command = "BACKWARD\n"
    elif right > 0:
        command = "RIGHT\n"
    elif right < 0:
        command = "LEFT\n"
    else:
        command = "STOP\n"

    serial_port.write(command.encode())
    print(f"Sent to serial: {command.strip()}")

def detect_persons(image):
    results = model(image)
    persons = []
    for det in results.xyxy[0]:
        class_id = int(det[5])
        if model.names[class_id] == 'person':
            x1, y1, x2, y2 = map(int, det[:4])
            area = (x2 - x1) * (y2 - y1)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            persons.append({'bbox': (x1, y1, x2, y2), 'cx': cx, 'cy': cy, 'area': area})
    return persons

def estimate_distance(cx_left, cx_right):
    disparity = float(cx_left - cx_right)
    if disparity > 0:
        return (focal_length_px * baseline_m) / disparity
    return None

def match_largest_person(persons_left, persons_right):
    if not persons_left or not persons_right:
        return None, None
    largest_left = max(persons_left, key=lambda p: p['area'])
    closest_match = min(persons_right, key=lambda p: abs(p['cy'] - largest_left['cy']))
    return largest_left, closest_match

def detect_and_follow():
    left_img = left_cam.capture_array()
    right_img = right_cam.capture_array()

    persons_left = detect_persons(left_img)
    persons_right = detect_persons(right_img)

    p_left, p_right = match_largest_person(persons_left, persons_right)

    if p_left and p_right:
        distance = estimate_distance(p_left['cx'], p_right['cx'])

        # Draw bounding box
        x1, y1, x2, y2 = p_left['bbox']
        cv2.rectangle(left_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{distance:.2f}m" if distance else "N/A"
        cv2.putText(left_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Movement decision
        if distance:
            offset = (p_left['cx'] - 320) / 320
            if abs(offset) > 0.2:
                send_motor_output(0, 1 if offset > 0 else -1)
            elif distance > 0.4:
                send_motor_output(1, 0)
            elif 0 < distance <= 0.4:
                send_motor_output(-1, 0)
            else:
                send_motor_output(0, 0)
        else:
            send_motor_output(0, 0)
    else:
        send_motor_output(0, 0)

    return left_img

@atexit.register
def cleanup():
    left_cam.stop()
    right_cam.stop()
    if serial_port.is_open:
        serial_port.close()
    print("Cameras and serial connection shut down.")

# Main loop
if __name__ == "__main__":
    try:
        while True:
            detect_and_follow()
            time.sleep(0.1)  # control frame rate
    except KeyboardInterrupt:
        print("Interrupted by user.")