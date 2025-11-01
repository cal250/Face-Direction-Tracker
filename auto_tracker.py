import cv2
import serial
import threading
import numpy as np
from collections import deque
import time

# === CONFIG ===
ARDUINO_PORT = "COM10"
BAUD_RATE = 115200
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

DEADZONE = 25
PAN_GAIN = 4.0
TILT_GAIN = 3.0
MAX_STEP = 30

class InstantTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(3, FRAME_WIDTH)
        self.cap.set(4, FRAME_HEIGHT)
        self.cap.set(5, 30)  # 30 FPS

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        self.ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0)
        time.sleep(1)

        self.command_queue = deque()
        self.running = True

        # Start serial thread
        threading.Thread(target=self.serial_worker, daemon=True).start()

    def serial_worker(self):
        while self.running:
            if self.command_queue:
                cmd = self.command_queue.popleft()
                try:
                    self.ser.write(cmd.encode())
                except:
                    pass
            time.sleep(0.001)  # 1ms loop

    def send(self, pan, tilt):
        if pan == 0 and tilt == 0: return
        cmd = f"P{pan},T{tilt}\n"
        self.command_queue.append(cmd)

    def run(self):
        print("INSTANT TRACKING STARTED (0ms LAG)")
        prev_cx = CENTER_X
        prev_cy = CENTER_Y

        while True:
            ret, frame = self.cap.read()
            if not ret: break
            frame = cv2.flip(frame, 1)

            # === PROCESS EVERY FRAME (NO SKIP) ===
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 3, minSize=(80, 80))

            if len(faces) > 0:
                x, y, w, h = max(faces, key=lambda r: r[2]*r[3])
                cx = x + w // 2
                cy = y + h // 2

                # === PREDICTIVE TRACKING ===
                vel_x = cx - prev_cx
                vel_y = cy - prev_cy
                pred_cx = cx + vel_x * 0.5  # Predict 0.5 frame ahead
                pred_cy = cy + vel_y * 0.5

                dx = pred_cx - CENTER_X
                dy = pred_cy - CENTER_Y

                pan_steps = tilt_steps = 0
                if abs(dx) > DEADZONE:
                    pan_steps = int(np.clip(dx * PAN_GAIN, -MAX_STEP, MAX_STEP))
                if abs(dy) > DEADZONE:
                    tilt_steps = int(np.clip(dy * TILT_GAIN, -MAX_STEP, MAX_STEP))

                self.send(pan_steps, tilt_steps)

                prev_cx, prev_cy = cx, cy

                # Visual feedback
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (int(pred_cx), int(pred_cy)), 6, (0, 255, 255), -1)
                cv2.putText(frame, "INSTANT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                self.send(0, 0)
                cv2.putText(frame, "NO FACE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # Center zone
            cv2.rectangle(frame, (CENTER_X-DEADZONE, CENTER_Y-DEADZONE),
                         (CENTER_X+DEADZONE, CENTER_Y+DEADZONE), (255, 0, 0), 1)

            cv2.imshow('ZERO LAG TRACKING', frame)
            if cv2.waitKey(1) == ord('q'):
                break

        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()
        self.ser.close()

if __name__ == "__main__":
    tracker = InstantTracker()
    tracker.run()