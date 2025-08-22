#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

/**
 * Camera Test for Garbage-Catching Dustbin
 * BeagleBone AI-64 Compatible
 * 
 * This script tests USB/CSI camera functionality and displays live video feed.
 * Press 'q' to quit, 's' to save a test frame, 'i' to show camera info.
 */

class CameraTest {
private:
    int cameraIndex;
    int width;
    int height;
    cv::VideoCapture cap;

public:
    /**
     * Constructor
     * @param camera_index Camera device index (0 for first camera)
     * @param w Frame width
     * @param h Frame height
     */
    CameraTest(int camera_index = 0, int w = 640, int h = 480) 
        : cameraIndex(camera_index), width(w), height(h) {}

    /**
     * Initialize and configure camera
     * @return true if successful, false otherwise
     */
    bool initializeCamera() {
        std::cout << "Initializing camera " << cameraIndex << "..." << std::endl;

        // Try to open camera
        cap.open(cameraIndex);

        if (!cap.isOpened()) {
            std::cout << "Error: Cannot open camera " << cameraIndex << std::endl;
            return false;
        }

        // Set camera properties
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS, 30);

        // Verify actual settings
        int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        double actualFps = cap.get(cv::CAP_PROP_FPS);

        std::cout << "Camera initialized successfully!" << std::endl;
        std::cout << "Resolution: " << actualWidth << "x" << actualHeight << std::endl;
        std::cout << "FPS: " << actualFps << std::endl;

        return true;
    }

    /**
     * Run camera test with live preview
     * @return true if successful, false otherwise
     */
    bool runTest() {
        if (!initializeCamera()) {
            return false;
        }

        std::cout << "\nCamera Test Running..." << std::endl;
        std::cout << "Press 'q' to quit" << std::endl;
        std::cout << "Press 's' to save current frame" << std::endl;
        std::cout << "Press 'i' to show camera info" << std::endl;

        int frameCount = 0;
        auto startTime = std::chrono::high_resolution_clock::now();
        cv::Mat frame;

        try {
            while (true) {
                // Capture frame
                bool ret = cap.read(frame);

                if (!ret) {
                    std::cout << "Error: Cannot read frame from camera" << std::endl;
                    break;
                }

                frameCount++;

                // Add overlay information
                addOverlayInfo(frame, frameCount, startTime);

                // Display frame
                cv::imshow("Camera Test - Garbage Dustbin", frame);

                // Handle keyboard input
                int key = cv::waitKey(1) & 0xFF;

                if (key == 'q' || key == 'Q') {
                    std::cout << "Quitting camera test..." << std::endl;
                    break;
                } else if (key == 's' || key == 'S') {
                    saveFrame(frame);
                } else if (key == 'i' || key == 'I') {
                    printCameraInfo();
                }
            }
        } catch (const std::exception& e) {
            std::cout << "\nCamera test interrupted: " << e.what() << std::endl;
        }

        cleanup();
        return true;
    }

    /**
     * Add informational overlay to frame
     * @param frame The frame to add overlay to
     * @param frameCount Current frame number
     * @param startTime Start time for FPS calculation
     */
    void addOverlayInfo(cv::Mat& frame, int frameCount, 
                       const std::chrono::high_resolution_clock::time_point& startTime) {
        
        // Calculate FPS
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
        double elapsedSeconds = elapsed.count() / 1000.0;
        double fps = (elapsedSeconds > 0) ? frameCount / elapsedSeconds : 0.0;

        // Font settings
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.6;
        cv::Scalar color(0, 255, 0);  // Green
        int thickness = 2;

        // FPS counter
        std::string fpsText = "FPS: " + std::to_string(fps).substr(0, 4);
        cv::putText(frame, fpsText, cv::Point(10, 30), font, fontScale, color, thickness);

        // Frame counter
        std::string frameText = "Frame: " + std::to_string(frameCount);
        cv::putText(frame, frameText, cv::Point(10, 60), font, fontScale, color, thickness);

        // Timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
        std::string timeText = "Time: " + ss.str();
        cv::putText(frame, timeText, cv::Point(10, 90), font, fontScale, color, thickness);

        // Resolution info
        int height = frame.rows;
        int width = frame.cols;
        std::string resText = "Res: " + std::to_string(width) + "x" + std::to_string(height);
        cv::putText(frame, resText, cv::Point(10, 120), font, fontScale, color, thickness);

        // Draw center crosshair for reference
        int centerX = width / 2;
        int centerY = height / 2;
        cv::drawMarker(frame, cv::Point(centerX, centerY), cv::Scalar(255, 255, 0),
                      cv::MARKER_CROSS, 20, 2);
    }

    /**
     * Save current frame as image
     * @param frame The frame to save
     */
    void saveFrame(const cv::Mat& frame) {
        // Generate timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        
        std::string filename = "camera_test_" + ss.str() + ".jpg";

        if (cv::imwrite(filename, frame)) {
            std::cout << "Frame saved as: " << filename << std::endl;
        } else {
            std::cout << "Error: Could not save frame" << std::endl;
        }
    }

    /**
     * Print detailed camera information
     */
    void printCameraInfo() {
        if (!cap.isOpened()) {
            return;
        }

        std::cout << "\n--- Camera Information ---" << std::endl;
        std::cout << "Width: " << static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)) << std::endl;
        std::cout << "Height: " << static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)) << std::endl;
        std::cout << "FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
        std::cout << "Brightness: " << cap.get(cv::CAP_PROP_BRIGHTNESS) << std::endl;
        std::cout << "Contrast: " << cap.get(cv::CAP_PROP_CONTRAST) << std::endl;
        std::cout << "Saturation: " << cap.get(cv::CAP_PROP_SATURATION) << std::endl;
        std::cout << "-------------------------\n" << std::endl;
    }

    /**
     * Clean up resources
     */
    void cleanup() {
        if (cap.isOpened()) {
            cap.release();
        }
        cv::destroyAllWindows();
        std::cout << "Camera resources cleaned up" << std::endl;
    }

    /**
     * Destructor
     */
    ~CameraTest() {
        cleanup();
    }
};

/**
 * Main function
 */
int main() {
    std::cout << "=== Garbage Dustbin Camera Test ===" << std::endl;
    std::cout << "BeagleBone AI-64 Compatible" << std::endl;
    std::cout << "===================================" << std::endl;

    // Try different camera indices if default doesn't work
    std::vector<int> cameraIndices = {0, 1, 2};
    bool success = false;

    for (int camIndex : cameraIndices) {
        std::cout << "\nTrying camera index " << camIndex << "..." << std::endl;
        
        CameraTest cameraTest(camIndex);
        
        if (cameraTest.runTest()) {
            success = true;
            break;
        } else {
            std::cout << "Camera " << camIndex << " failed, trying next..." << std::endl;
        }
    }

    if (!success) {
        std::cout << "Error: No working camera found!" << std::endl;
        return 1;
    }

    std::cout << "Camera test completed successfully!" << std::endl;
    return 0;
}
