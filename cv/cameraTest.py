#!/usr/bin/env python3
"""
Script 1: Camera Test for Garbage-Catching Dustbin
BeagleBone AI-64 Compatible

This script tests USB/CSI camera functionality and displays live video feed.
Press 'q' to quit, 's' to save a test frame.
"""

import cv2
import sys
import time
from datetime import datetime


class CameraTest:
    def __init__(self, camera_index=0, width=640, height=480):
        """
        Initialize camera test

        Args:
            camera_index (int): Camera device index (0 for first camera)
            width (int): Frame width
            height (int): Frame height
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None

    def initialize_camera(self):
        """Initialize and configure camera"""
        print(f"Initializing camera {self.camera_index}...")

        # Try to open camera
        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            print(f"Error: Cannot open camera {self.camera_index}")
            return False

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Verify actual settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        print(f"Camera initialized successfully!")
        print(f"Resolution: {actual_width}x{actual_height}")
        print(f"FPS: {actual_fps}")

        return True

    def run_test(self):
        """Run camera test with live preview"""
        if not self.initialize_camera():
            return False

        print("\nCamera Test Running...")
        print("Press 'q' to quit")
        print("Press 's' to save current frame")
        print("Press 'i' to show camera info")

        frame_count = 0
        start_time = time.time()

        try:
            while True:
                # Capture frame
                ret, frame = self.cap.read()

                if not ret:
                    print("Error: Cannot read frame from camera")
                    break

                frame_count += 1

                # Add overlay information
                self.add_overlay_info(frame, frame_count, start_time)

                # Display frame
                cv2.imshow('Camera Test - Garbage Dustbin', frame)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    print("Quitting camera test...")
                    break
                elif key == ord('s'):
                    self.save_frame(frame)
                elif key == ord('i'):
                    self.print_camera_info()

        except KeyboardInterrupt:
            print("\nCamera test interrupted by user")

        finally:
            self.cleanup()

        return True

    def add_overlay_info(self, frame, frame_count, start_time):
        """Add informational overlay to frame"""
        # Calculate FPS
        elapsed_time = time.time() - start_time
        if elapsed_time > 0:
            fps = frame_count / elapsed_time
        else:
            fps = 0

        # Add text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)  # Green
        thickness = 2

        # FPS counter
        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                    font, font_scale, color, thickness)

        # Frame counter
        cv2.putText(frame, f'Frame: {frame_count}', (10, 60),
                    font, font_scale, color, thickness)

        # Timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, f'Time: {timestamp}', (10, 90),
                    font, font_scale, color, thickness)

        # Resolution info
        height, width = frame.shape[:2]
        cv2.putText(frame, f'Res: {width}x{height}', (10, 120),
                    font, font_scale, color, thickness)

        # Draw center crosshair for reference
        center_x, center_y = width // 2, height // 2
        cv2.drawMarker(frame, (center_x, center_y), (255, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

    def save_frame(self, frame):
        """Save current frame as image"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"camera_test_{timestamp}.jpg"

        if cv2.imwrite(filename, frame):
            print(f"Frame saved as: {filename}")
        else:
            print("Error: Could not save frame")

    def print_camera_info(self):
        """Print detailed camera information"""
        if self.cap is None:
            return

        print("\n--- Camera Information ---")
        print(f"Width: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}")
        print(f"Height: {int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        print(f"FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
        print(f"Brightness: {self.cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
        print(f"Contrast: {self.cap.get(cv2.CAP_PROP_CONTRAST)}")
        print(f"Saturation: {self.cap.get(cv2.CAP_PROP_SATURATION)}")
        print("-------------------------\n")

    def cleanup(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Camera resources cleaned up")


def main():
    """Main function"""
    print("=== Garbage Dustbin Camera Test ===")
    print("BeagleBone AI-64 Compatible")
    print("=" * 35)

    # Create and run camera test
    # Try different camera indices if default doesn't work
    for cam_index in [0, 1, 2]:
        print(f"\nTrying camera index {cam_index}...")
        camera_test = CameraTest(camera_index=cam_index)

        if camera_test.run_test():
            break
        else:
            print(f"Camera {cam_index} failed, trying next...")
            camera_test.cleanup()
    else:
        print("Error: No working camera found!")
        sys.exit(1)

    print("Camera test completed successfully!")


if __name__ == "__main__":
    main()
