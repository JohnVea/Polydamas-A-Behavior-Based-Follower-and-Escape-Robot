import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2
from flask import Flask, render_template, Response
import atexit
from flask import Flask, render_template, Response, request, jsonify

# Global HSV values (initial ranges for red)
hsv_ranges = {
    'lh': 0, 'uh': 10,
    'ls': 100, 'us': 255,
    'lv': 100, 'uv': 255
}

# Flask app
app = Flask(__name__)

# Serial setup
serial_port = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    timeout=1
)

# Camera setup
left_cam = Picamera2(0)
right_cam = Picamera2(1)
for cam in [left_cam, right_cam]:
    cam.preview_configuration.main.size = (640, 480)
    cam.preview_configuration.main.format = "RGB888"
    cam.configure("preview")
    cam.start()

# Stereo calibration
focal_length_px = 1029  # Replace with actual value
baseline_m = 0.05       # Distance between cameras in meters

def send_motor_output(forward, right):
    if forward > 0:
        command = "FORWARD\n"
    elif forward < 0:
        command = "BACKWARD\n"
    elif right > 0:
        command = " RIGHT\n"
    elif right < 0:
        command = " LEFT\n"
    else:
        command = " STOP\n"
    serial_port.write(command.encode())
    print(f"Sent to serial: {command.strip()}")

def detect_red_blob(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_red = np.array([hsv_ranges['lh'], hsv_ranges['ls'], hsv_ranges['lv']])
    upper_red = np.array([hsv_ranges['uh'], hsv_ranges['us'], hsv_ranges['uv']])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area > 500:
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                return {'cx': cx, 'cy': cy, 'area': area, 'contour': largest}
    return None

def estimate_distance(cx_left, cx_right):
    disparity = float(cx_left - cx_right)
    if disparity > 0:
        return (focal_length_px * baseline_m) / disparity
    return None

def detect_and_follow():
    left_img = left_cam.capture_array()
    right_img = right_cam.capture_array()

    red_left = detect_red_blob(left_img)
    red_right = detect_red_blob(right_img)

    if red_left and red_right:
        distance = estimate_distance(red_left['cx'], red_right['cx'])

        # Draw bounding info
        cv2.drawContours(left_img, [red_left['contour']], -1, (0, 255, 0), 2)
        cv2.circle(left_img, (red_left['cx'], red_left['cy']), 5, (255, 255, 255), -1)
        label = f"{distance:.2f}m" if distance else "N/A"
        cv2.putText(left_img, label, (red_left['cx'], red_left['cy'] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Movement logic
        offset = (red_left['cx'] - 320) / 320
        if abs(offset) > 0.2:
            send_motor_output(0, 1 if offset > 0 else -1)
        elif distance and distance > 0.4:
            send_motor_output(-1, 0)
        elif distance and distance <= 0.4:
            send_motor_output(1, 0)
        else:
            send_motor_output(0, 0)
    else:
        send_motor_output(0, 0)

    return left_img

def generate_frames():
    while True:
        frame = detect_and_follow()
        ret, buffer = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        if not ret:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n\r\n')

def generate_mask_frames():
    while True:
        img = left_cam.capture_array()
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        lower_red = np.array([hsv_ranges['lh'], hsv_ranges['ls'], hsv_ranges['lv']])
        upper_red = np.array([hsv_ranges['uh'], hsv_ranges['us'], hsv_ranges['uv']])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)  # convert for streaming
        ret, buffer = cv2.imencode('.jpg', mask_rgb)
        if not ret:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n\r\n')


@app.route('/')
def index():
    return render_template('index.html')  # You can create this HTML template

@app.route('/update_hsv', methods=['POST'])
def update_hsv():
    data = request.get_json()
    for key in hsv_ranges:
        if key in data:
            hsv_ranges[key] = int(data[key])
    return jsonify(success=True)

@app.route('/video_mask')
def video_mask():
    return Response(generate_mask_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@atexit.register
def cleanup():
    left_cam.stop()
    right_cam.stop()
    if serial_port.is_open:
        serial_port.close()
    print("Cameras and serial connection shut down.")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
