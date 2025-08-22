#!/usr/bin/env python3
"""
Script 2: Background Subtraction Object Detection for Garbage-Catching Dustbin
BeagleBone AI-64 Compatible

This script detects moving objects (garbage) by ignoring static background.
Uses MOG2 background subtractor for robust detection.
"""

import cv2
import numpy as np
import sys
import time
from datetime import datetime


class GarbageDetector:
    def __init__(self, camera_index=0, width=640, height=480):
        """
        Initialize garbage detector with background subtraction

        Args:
            camera_index (int): Camera device index
            width (int): Frame width
            height (int): Frame height
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None

        # Background subtractor (MOG2 - Mixture of Gaussians)
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            detectShadows=True,
            varThreshold=50,      # Threshold for pixel classification
            history=500           # Number of frames to build background model
        )

        # Detection parameters
        self.min_contour_area = 500     # Minimum area to consider as object
        self.max_contour_area = 50000   # Maximum area to avoid false positives
        # Background learning rate (lower = slower adaptation)
        self.learning_rate = 0.01

        # Morphological operations kernels
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_close = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (7, 7))

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

    def initialize_camera(self):
        """Initialize camera"""
        print(f"Initializing camera {self.camera_index}...")

        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            print(f"Error: Cannot open camera {self.camera_index}")
            return False

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        print("Camera initialized for garbage detection!")
        return True

    def preprocess_frame(self, frame):
        """
        Preprocess frame for better detection

        Args:
            frame: Input frame

        Returns:
            preprocessed frame
        """
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        return blurred

    def detect_objects(self, frame):
        """
        Detect moving objects using background subtraction

        Args:
            frame: Input frame

        Returns:
            List of detected objects with their properties
        """
        # Preprocess frame
        processed_frame = self.preprocess_frame(frame)

        # Apply background subtraction
        fg_mask = self.bg_subtractor.apply(
            processed_frame, learningRate=self.learning_rate)

        # Remove shadows (optional, shadows are marked as 127 in MOG2)
        fg_mask[fg_mask == 127] = 0

        # Morphological operations to clean up the mask
        # Opening: removes noise
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel_open)
        # Closing: fills holes
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel_close)

        # Find contours
        contours, _ = cv2.findContours(
            fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []

        for contour in contours:
            # Calculate contour area
            area = cv2.contourArea(contour)

            # Filter by area
            if self.min_contour_area < area < self.max_contour_area:
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate centroid
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = x + w//2, y + h//2

                # Create object dictionary
                obj = {
                    'bbox': (x, y, w, h),
                    'centroid': (cx, cy),
                    'area': area,
                    'contour': contour
                }

                detected_objects.append(obj)

        return detected_objects, fg_mask

    def draw_detections(self, frame, detected_objects):
        """
        Draw detection results on frame

        Args:
            frame: Input frame
            detected_objects: List of detected objects
        """
        frame_height, frame_width = frame.shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2

        # Draw center reference point
        cv2.drawMarker(frame, (center_x, center_y), (255, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

        for i, obj in enumerate(detected_objects):
            x, y, w, h = obj['bbox']
            cx, cy = obj['centroid']
            area = obj['area']

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw centroid
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

            # Draw line from center to centroid
            cv2.line(frame, (center_x, center_y), (cx, cy), (255, 0, 255), 2)

            # Add object information
            info_text = f"Obj{i+1}: Area={int(area)}"
            cv2.putText(frame, info_text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Add centroid coordinates
            coord_text = f"({cx}, {cy})"
            cv2.putText(frame, coord_text, (cx - 30, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        return frame

    def add_overlay_info(self, frame, fps):
        """Add informational overlay"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)  # White
        thickness = 2

        # Background rectangle for better text visibility
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (300, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        # Add information text
        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                    font, font_scale, color, thickness)
        cv2.putText(frame, f'Frame: {self.frame_count}', (10, 55),
                    font, font_scale, color, thickness)
        cv2.putText(frame, f'Detections: {self.detection_count}', (10, 80),
                    font, font_scale, color, thickness)
        cv2.putText(frame, f'Learning Rate: {self.learning_rate:.3f}', (10, 105),
                    font, font_scale, color, thickness)
        cv2.putText(frame, 'Press: r-reset, +/- learning rate, q-quit', (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def run_detection(self):
        """Run garbage detection system"""
        if not self.initialize_camera():
            return False

        print("\nGarbage Detection System Running...")
        print("Building background model... Please keep the area clear for initial frames")
        print("Controls:")
        print("  'q' - Quit")
        print("  'r' - Reset background model")
        print("  '+' - Increase learning rate")
        print("  '-' - Decrease learning rate")
        print("  's' - Save current frame")

        start_time = time.time()

        try:
            while True:
                ret, frame = self.cap.read()

                if not ret:
                    print("Error: Cannot read frame")
                    break

                self.frame_count += 1

                # Detect objects
                detected_objects, fg_mask = self.detect_objects(frame)
                self.detection_count = len(detected_objects)

                # Draw detections
                display_frame = self.draw_detections(
                    frame.copy(), detected_objects)

                # Calculate FPS
                elapsed_time = time.time() - start_time
                fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0

                # Add overlay information
                self.add_overlay_info(display_frame, fps)

                # Show frames
                cv2.imshow('Garbage Detection - Original', display_frame)
                cv2.imshow('Garbage Detection - Foreground Mask', fg_mask)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('r'):
                    print("Resetting background model...")
                    self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                        detectShadows=True, varThreshold=50, history=500)
                elif key == ord('+') or key == ord('='):
                    self.learning_rate = min(0.1, self.learning_rate + 0.01)
                    print(
                        f"Learning rate increased to: {self.learning_rate:.3f}")
                elif key == ord('-') or key == ord('_'):
                    self.learning_rate = max(0.001, self.learning_rate - 0.01)
                    print(
                        f"Learning rate decreased to: {self.learning_rate:.3f}")
                elif key == ord('s'):
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"detection_{timestamp}.jpg", display_frame)
                    cv2.imwrite(f"mask_{timestamp}.jpg", fg_mask)
                    print(f"Frames saved with timestamp: {timestamp}")

        except KeyboardInterrupt:
            print("\nDetection interrupted by user")

        finally:
            self.cleanup()

        return True

    def cleanup(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Detection system cleaned up")


def main():
    """Main function"""
    print("=== Garbage Detection with Background Subtraction ===")
    print("BeagleBone AI-64 Compatible")
    print("=" * 50)

    # Create and run detector
    detector = GarbageDetector(camera_index=0)

    if not detector.run_detection():
        print("Error: Detection system failed to start!")
        sys.exit(1)

    print("Garbage detection completed successfully!")


if __name__ == "__main__":
    main()
