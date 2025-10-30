import cv2
import numpy as np
import time

# Load Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

# Check if cascade loaded
if face_cascade.empty():
    raise IOError("Failed to load cascade classifier")

# Video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Variables for tracking
prev_center = None
prev_time = time.time()
movement_threshold = 5  # Minimum pixel movement to register direction
fps = 0

# For FPS calculation
prev_frame_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Flip frame horizontally for mirror effect
    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100)
    )

    current_time = time.time()
    delta_time = current_time - prev_time

    direction = ""
    speed = 0

    if len(faces) > 0:
        # Get the largest face (most likely the closest)
        x, y, w, h = max(faces, key=lambda rect: rect[2] * rect[3])
        cx = x + w // 2
        cy = y + h // 2

        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if prev_center is not None and delta_time > 0:
            dx = cx - prev_center[0]
            dy = cy - prev_center[1]

            # Calculate speed in px/s
            distance = np.sqrt(dx**2 + dy**2)
            if delta_time > 0:
                speed = distance / delta_time  # px/s

            # Determine direction
            if abs(dx) > movement_threshold or abs(dy) > movement_threshold:
                if abs(dx) > abs(dy):
                    direction = "Right" if dx > 0 else "Left"
                else:
                    direction = "Down" if dy > 0 else "Up"

                # Diagonal cases (optional refinement)
                if abs(dx) > movement_threshold and abs(dy) > movement_threshold:
                    if dx > 0 and dy > 0:
                        direction = "Down-Right"
                    elif dx > 0 and dy < 0:
                        direction = "Up-Right"
                    elif dx < 0 and dy > 0:
                        direction = "Down-Left"
                    elif dx < 0 and dy < 0:
                        direction = "Up-Left"

        # Update previous center
        prev_center = (cx, cy)
        prev_time = current_time

        # Display center point
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

        # Display direction and speed
        text = f"{direction} | {speed:.1f} px/s"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2)

    else:
        prev_center = None  # Reset when no face

    # Calculate and display FPS
    current_frame_time = time.time()
    fps = 1 / (current_frame_time - prev_frame_time) if (current_frame_time - prev_frame_time) > 0 else 0
    prev_frame_time = current_frame_time
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 0), 2)

    # Show frame
    cv2.imshow('Face Direction Tracker', frame)

    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()