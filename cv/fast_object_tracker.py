import cv2
import numpy as np
from collections import deque
import time
import math

# --- CONFIGURATION VARIABLES ---

# Camera and Frame Settings
CAMERA_INDEX = 2  # 2 for your camera, or 0 for default, or path to video file
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Background Subtractor Settings
BG_HISTORY = 500
BG_VAR_THRESHOLD = 50

# Object and Movement Thresholds
MIN_CONTOUR_AREA = 500  # Minimum area to be considered a valid object

# --- Trajectory Tracking Settings ---
MAX_TRAJECTORY_POINTS = 10  # Number of points to keep for trajectory calculation
MIN_POINTS_FOR_PREDICTION = 3  # Minimum points needed for trajectory prediction
PREDICTION_TIME = 1.0  # Time in seconds to predict ahead

# --- Control Logic Thresholds for Differential Drive Robot ---
CENTER_X_THRESHOLD = 80  # in pixels
IDEAL_AREA = 10000  # Desired object area (distance)
AREA_THRESHOLD = 2000

# Robot movement parameters
ROBOT_SPEED = 100  # pixels per second (approximate robot movement speed)
INTERCEPT_THRESHOLD = 50  # pixels - how close we need to be to intercept point

class TrajectoryTracker:
    def __init__(self, max_points=MAX_TRAJECTORY_POINTS):
        self.positions = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.velocity = (0, 0)
        self.acceleration = (0, 0)

    def add_point(self, x, y, timestamp):
        """Add a new position point with timestamp"""
        self.positions.append((x, y))
        self.timestamps.append(timestamp)
        self._calculate_motion_parameters()

    def _calculate_motion_parameters(self):
        """Calculate velocity and acceleration from recent points"""
        if len(self.positions) < 2:
            return

        # Calculate velocity (pixels per second)
        recent_positions = list(self.positions)[-3:]  # Use last 3 points for stability
        recent_times = list(self.timestamps)[-3:]

        if len(recent_positions) >= 2:
            dt = recent_times[-1] - recent_times[-2]
            if dt > 0:
                dx = recent_positions[-1][0] - recent_positions[-2][0]
                dy = recent_positions[-1][1] - recent_positions[-2][1]
                self.velocity = (dx / dt, dy / dt)

        # Calculate acceleration if we have enough points
        if len(recent_positions) >= 3:
            dt1 = recent_times[-1] - recent_times[-2]
            dt2 = recent_times[-2] - recent_times[-3]
            if dt1 > 0 and dt2 > 0:
                # Previous velocity
                prev_vx = (recent_positions[-2][0] - recent_positions[-3][0]) / dt2
                prev_vy = (recent_positions[-2][1] - recent_positions[-3][1]) / dt2

                # Acceleration
                ax = (self.velocity[0] - prev_vx) / dt1
                ay = (self.velocity[1] - prev_vy) / dt1
                self.acceleration = (ax, ay)

    def predict_position(self, time_ahead):
        """Predict object position after time_ahead seconds"""
        if len(self.positions) < MIN_POINTS_FOR_PREDICTION:
            return None

        current_pos = self.positions[-1]
        vx, vy = self.velocity
        ax, ay = self.acceleration

        # Use kinematic equation: s = ut + 0.5*a*t^2
        predicted_x = current_pos[0] + vx * time_ahead + 0.5 * ax * time_ahead**2
        predicted_y = current_pos[1] + vy * time_ahead + 0.5 * ay * time_ahead**2

        return (int(predicted_x), int(predicted_y))

    def get_intercept_point(self, robot_pos, robot_speed):
        """Calculate the optimal intercept point considering robot movement time"""
        if len(self.positions) < MIN_POINTS_FOR_PREDICTION:
            return None

        current_obj_pos = self.positions[-1]

        # Iterative approach to find intercept point
        for t in np.arange(0.1, 5.0, 0.1):  # Check up to 5 seconds ahead
            predicted_obj_pos = self.predict_position(t)
            if predicted_obj_pos is None:
                continue

            # Calculate distance robot needs to travel
            distance_to_intercept = math.sqrt(
                (predicted_obj_pos[0] - robot_pos[0])**2 +
                (predicted_obj_pos[1] - robot_pos[1])**2
            )

            # Time for robot to reach intercept point
            robot_travel_time = distance_to_intercept / robot_speed

            # If robot can reach the point in roughly the same time
            if abs(robot_travel_time - t) < 0.2:  # 0.2 second tolerance
                return predicted_obj_pos, t

        return None, None

def calculate_robot_command(current_pos, target_pos, frame_center_x, current_area):
    """Calculate robot command to reach target intercept point"""
    if target_pos is None:
        return "stop"

    target_x, target_y = target_pos
    robot_x = frame_center_x  # Assume robot is at frame center

    # Distance to target
    dx = target_x - robot_x
    dy = target_y - current_pos[1] if current_pos else 0

    # Priority 1: Rotate to face the target
    if abs(dx) > CENTER_X_THRESHOLD:
        if dx < 0:
            return "turn left"
        else:
            return "turn right"

    # Priority 2: Move forward/backward to intercept
    # If target is ahead and we're not too close
    if current_area < IDEAL_AREA - AREA_THRESHOLD:
        return "move forward fast"  # Fast movement to intercept
    elif current_area > IDEAL_AREA + AREA_THRESHOLD:
        return "move backward"

    return "stop"

def main():
    """
    Main function with trajectory prediction for object catching
    """
    # Initialize video capture
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open camera index {CAMERA_INDEX}.")
        return

    # Set frame dimensions
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Get the actual frame center (robot position)
    frame_center_x = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
    frame_center_y = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2
    robot_pos = (frame_center_x, frame_center_y)

    # Initialize background subtractor and trajectory tracker
    fgbg = cv2.createBackgroundSubtractorMOG2(history=BG_HISTORY, varThreshold=BG_VAR_THRESHOLD, detectShadows=False)
    tracker = TrajectoryTracker()

    print("Starting enhanced robot control with trajectory prediction... Press 'q' to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        current_time = time.time()

        # Apply background subtraction
        fgmask = fgbg.apply(frame)

        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        command = "stop"  # Default command
        current_pos = None
        current_area = 0

        if contours:
            # Find the largest contour
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_CONTOUR_AREA:
                # Get object position
                x, y, w, h = cv2.boundingRect(c)
                object_center_x = x + w // 2
                object_center_y = y + h // 2
                current_pos = (object_center_x, object_center_y)
                current_area = area

                # Add point to trajectory tracker
                tracker.add_point(object_center_x, object_center_y, current_time)

                # Get intercept point
                intercept_result = tracker.get_intercept_point(robot_pos, ROBOT_SPEED)

                if intercept_result is not None:
                    intercept_point, intercept_time = intercept_result
                else:
                    intercept_point, intercept_time = None, None

                # Calculate command based on intercept strategy
                if intercept_point is not None:
                    command = calculate_robot_command(current_pos, intercept_point, frame_center_x, current_area)

                    # Visualize intercept point
                    cv2.circle(frame, intercept_point, 15, (255, 0, 255), 3)
                    cv2.putText(frame, f"Intercept in {intercept_time:.1f}s",
                              (intercept_point[0] - 60, intercept_point[1] - 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                else:
                    # Fallback to basic tracking if prediction fails
                    if object_center_x < frame_center_x - CENTER_X_THRESHOLD:
                        command = "turn left"
                    elif object_center_x > frame_center_x + CENTER_X_THRESHOLD:
                        command = "turn right"
                    elif area < IDEAL_AREA - AREA_THRESHOLD:
                        command = "move forward"
                    elif area > IDEAL_AREA + AREA_THRESHOLD:
                        command = "move backward"

                # --- Visualization ---
                # Draw current object
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (object_center_x, object_center_y), 7, (0, 0, 255), -1)

                # Draw trajectory
                if len(tracker.positions) > 1:
                    points = list(tracker.positions)
                    for i in range(1, len(points)):
                        cv2.line(frame, points[i-1], points[i], (0, 255, 255), 2)

                # Draw predicted path
                if len(tracker.positions) >= MIN_POINTS_FOR_PREDICTION:
                    for t in np.arange(0.2, 2.0, 0.2):
                        pred_pos = tracker.predict_position(t)
                        if pred_pos:
                            cv2.circle(frame, pred_pos, 3, (255, 255, 0), -1)

                # Display motion info
                cv2.putText(frame, f"Area: {int(area)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, f"Velocity: ({tracker.velocity[0]:.1f}, {tracker.velocity[1]:.1f})",
                          (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Output command
        print(f"Command: {command}")

        # Display command and robot position
        cv2.putText(frame, f"COMMAND: {command.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Draw robot position (center of frame)
        cv2.circle(frame, robot_pos, 10, (255, 255, 255), -1)
        cv2.putText(frame, "ROBOT", (robot_pos[0] - 25, robot_pos[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw center threshold lines
        cv2.line(frame, (frame_center_x - CENTER_X_THRESHOLD, 0), (frame_center_x - CENTER_X_THRESHOLD, FRAME_HEIGHT), (255, 255, 0), 1)
        cv2.line(frame, (frame_center_x + CENTER_X_THRESHOLD, 0), (frame_center_x + CENTER_X_THRESHOLD, FRAME_HEIGHT), (255, 255, 0), 1)

        # Show results
        cv2.imshow("Robot Control with Trajectory Prediction", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Enhanced tracking script stopped.")

if __name__ == "__main__":
    main()
