import cv2
import numpy as np

# Open video source (0 for webcam, or replace with video file path)
cap = cv2.VideoCapture(2)

# Background subtractor
fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

prev_center = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Resize for speed (optional)
    frame = cv2.resize(frame, (640, 480))

    # Apply background subtraction
    fgmask = fgbg.apply(frame)

    # Remove noise
    kernel = np.ones((5, 5), np.uint8)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get largest contour
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > 300:  # filter out tiny noise
            x, y, w, h = cv2.boundingRect(c)
            center = (x + w // 2, y + h // 2)

            # Draw bounding box and center
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Compare with previous center → direction
            if prev_center is not None:
                dx = center[0] - prev_center[0]
                dy = center[1] - prev_center[1]

                direction = ""
                if abs(dx) > 10:
                    direction += "Right" if dx > 0 else "Left"
                if abs(dy) > 10:
                    direction += " Down" if dy > 0 else " Up"

                if direction:
                    cv2.putText(frame, f"Moving: {direction}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            prev_center = center

    # Show results
    cv2.imshow("Frame", frame)
    cv2.imshow("FG Mask", fgmask)

    # Exit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
