#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

// TI TDA4VM specific includes
#ifdef TI_TDA4VM_PLATFORM
#include <VX/vx.h>
#include <TI/tivx.h>
#include <TI/tivx_target_kernel.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#endif

// ARM NEON optimizations
#ifdef __aarch64__
#include <arm_neon.h>
#endif

// GStreamer for hardware-accelerated video pipeline
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

/**
 * TI TDA4VM Optimized Camera Test for BeagleBone AI-64
 * 
 * Hardware Acceleration Strategy:
 * - A72 cores: Main application logic, OpenCV operations
 * - C7x DSPs: Image preprocessing, filtering, feature extraction
 * - MMA: Deep learning inference (if needed)
 * - R5F cores: Real-time control tasks
 * - Hardware video decoders: Camera input processing
 * 
 * Pipeline: GStreamer → OpenVX → OpenCV interop
 */

class TDA4VMCameraTest {
private:
    // Camera configuration
    int cameraIndex;
    int width;
    int height;
    int targetFps;
    
    // OpenCV capture (fallback)
    cv::VideoCapture cap;
    
    // GStreamer pipeline for hardware acceleration
    GstElement *pipeline;
    GstElement *appsink;
    bool useGStreamerPipeline;
    
    // OpenVX context for TI acceleration
    #ifdef TI_TDA4VM_PLATFORM
    vx_context vxContext;
    vx_graph vxGraph;
    vx_image vxInputImage;
    vx_image vxOutputImage;
    #endif
    
    // Multi-threading components
    std::queue<cv::Mat> frameQueue;
    std::mutex queueMutex;
    std::condition_variable queueCondition;
    bool stopProcessing;
    std::thread captureThread;
    std::thread processThread;
    std::thread acceleratorThread;
    
    // Performance monitoring
    std::chrono::high_resolution_clock::time_point startTime;
    int totalFrames;
    int processedFrames;
    int acceleratedFrames;
    
    // Processing modes
    enum ProcessingMode {
        CPU_ONLY,           // A72 cores only
        DSP_ACCELERATED,    // C7x DSP acceleration
        FULL_HARDWARE       // GStreamer + OpenVX + DSP
    };
    ProcessingMode currentMode;
    
    // Performance statistics
    double avgCaptureLatency;
    double avgProcessLatency;
    double avgAcceleratorLatency;

public:
    TDA4VMCameraTest(int camera_index = 0, int w = 640, int h = 480, int fps = 30) 
        : cameraIndex(camera_index), width(w), height(h), targetFps(fps),
          pipeline(nullptr), appsink(nullptr), useGStreamerPipeline(false),
          stopProcessing(false), totalFrames(0), processedFrames(0), acceleratedFrames(0),
          currentMode(CPU_ONLY), avgCaptureLatency(0), avgProcessLatency(0), avgAcceleratorLatency(0) {
        
        std::cout << "TI TDA4VM Camera Test - BeagleBone AI-64" << std::endl;
        std::cout << "Jacinto 7 SoC Hardware Acceleration" << std::endl;
        
        detectTDA4VMCapabilities();
        initializeGStreamer();
        
        #ifdef TI_TDA4VM_PLATFORM
        initializeOpenVX();
        #endif
    }

    /**
     * Detect TI TDA4VM hardware capabilities
     */
    void detectTDA4VMCapabilities() {
        std::cout << "\n=== TI TDA4VM Hardware Detection ===" << std::endl;
        
        // Check CPU topology
        std::cout << "ARM Cortex-A72 cores: ";
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        int coreCount = 0;
        while (std::getline(cpuinfo, line)) {
            if (line.find("processor") != std::string::npos) {
                coreCount++;
            }
        }
        std::cout << coreCount << std::endl;
        
        // Check for DSP availability
        std::cout << "C7x DSP cores: ";
        if (std::ifstream("/dev/rpmsg_kdrv").good()) {
            std::cout << "Available (via RPMsg)" << std::endl;
        } else {
            std::cout << "Not accessible" << std::endl;
        }
        
        // Check for TI vision apps
        std::cout << "TI EdgeAI Apps: ";
        if (system("which edgeai-test-app > /dev/null 2>&1") == 0) {
            std::cout << "Installed" << std::endl;
        } else {
            std::cout << "Not found" << std::endl;
        }
        
        #ifdef TI_TDA4VM_PLATFORM
        std::cout << "OpenVX/TIOVX: Available" << std::endl;
        #else
        std::cout << "OpenVX/TIOVX: Not available (compile with TI SDK)" << std::endl;
        #endif
        
        // Check NEON support
        #ifdef __aarch64__
        std::cout << "ARM NEON SIMD: Available" << std::endl;
        #else
        std::cout << "ARM NEON SIMD: Not available" << std::endl;
        #endif
        
        std::cout << "====================================\n" << std::endl;
    }

    /**
     * Initialize GStreamer for hardware-accelerated video pipeline
     */
    bool initializeGStreamer() {
        gst_init(nullptr, nullptr);
        
        // Create GStreamer pipeline for TI ISP acceleration
        std::stringstream pipelineStr;
        pipelineStr << "v4l2src device=/dev/video" << cameraIndex 
                   << " ! video/x-raw,format=NV12,width=" << width 
                   << ",height=" << height << ",framerate=" << targetFps << "/1"
                   << " ! tiovxisp sensor-name=SENSOR_SONY_IMX219_RPI"  // TI ISP acceleration
                   << " ! video/x-raw,format=NV12"
                   << " ! tiovxmultiscaler"  // Hardware scaler
                   << " ! video/x-raw,format=RGB"
                   << " ! videoconvert ! appsink name=sink";
        
        pipeline = gst_parse_launch(pipelineStr.str().c_str(), nullptr);
        if (!pipeline) {
            std::cout << "Warning: Failed to create GStreamer pipeline, falling back to OpenCV" << std::endl;
            return false;
        }
        
        appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        if (!appsink) {
            std::cout << "Warning: Failed to get appsink, falling back to OpenCV" << std::endl;
            gst_object_unref(pipeline);
            return false;
        }
        
        // Configure appsink
        g_object_set(G_OBJECT(appsink), "emit-signals", TRUE, "sync", FALSE, nullptr);
        
        useGStreamerPipeline = true;
        std::cout << "GStreamer pipeline initialized with TI ISP acceleration" << std::endl;
        return true;
    }

    #ifdef TI_TDA4VM_PLATFORM
    /**
     * Initialize OpenVX context for TI accelerators
     */
    bool initializeOpenVX() {
        // Initialize TIOVX
        tivxInit();
        tivxHostInit();
        
        // Create OpenVX context
        vxContext = vxCreateContext();
        if (vxGetStatus((vx_reference)vxContext) != VX_SUCCESS) {
            std::cout << "Error: Failed to create OpenVX context" << std::endl;
            return false;
        }
        
        // Load TI kernels
        tivxRegisterOpenVXCoreKernels();
        tivxRegisterImagingKernels(vxContext);
        tivxRegisterHwaKernels(vxContext);
        
        // Create OpenVX graph for processing pipeline
        vxGraph = vxCreateGraph(vxContext);
        
        // Create image objects
        vxInputImage = vxCreateImage(vxContext, width, height, VX_DF_IMAGE_RGB);
        vxOutputImage = vxCreateImage(vxContext, width, height, VX_DF_IMAGE_RGB);
        
        std::cout << "OpenVX initialized with TI accelerators" << std::endl;
        return true;
    }
    #endif

    /**
     * Initialize camera with optimal settings for TDA4VM
     */
    bool initializeCamera() {
        std::cout << "Initializing TDA4VM optimized camera..." << std::endl;

        if (useGStreamerPipeline) {
            // Start GStreamer pipeline
            GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
            if (ret == GST_STATE_CHANGE_FAILURE) {
                std::cout << "Failed to start GStreamer pipeline" << std::endl;
                useGStreamerPipeline = false;
            } else {
                std::cout << "GStreamer pipeline started with hardware acceleration" << std::endl;
                return true;
            }
        }
        
        // Fallback to OpenCV with V4L2 optimizations
        cap.open(cameraIndex, cv::CAP_V4L2);
        
        if (!cap.isOpened()) {
            std::cout << "Error: Cannot open camera " << cameraIndex << std::endl;
            return false;
        }

        // Optimize for TDA4VM
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS, targetFps);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Minimize latency
        
        // Try to use hardware-friendly formats
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N','V','1','2'));  // NV12 for ISP
        
        int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        double actualFps = cap.get(cv::CAP_PROP_FPS);

        std::cout << "OpenCV V4L2 camera initialized" << std::endl;
        std::cout << "Resolution: " << actualWidth << "x" << actualHeight << std::endl;
        std::cout << "Target FPS: " << actualFps << std::endl;

        return true;
    }

    /**
     * Hardware-accelerated frame capture using GStreamer or V4L2
     */
    void captureFrames() {
        cv::Mat frame;
        auto frameStart = std::chrono::high_resolution_clock::now();
        
        while (!stopProcessing) {
            auto captureStart = std::chrono::high_resolution_clock::now();
            
            bool frameReceived = false;
            
            if (useGStreamerPipeline) {
                // Get frame from GStreamer pipeline
                GstSample *sample;
                g_signal_emit_by_name(appsink, "pull-sample", &sample);
                
                if (sample) {
                    GstBuffer *buffer = gst_sample_get_buffer(sample);
                    GstMapInfo map;
                    gst_buffer_map(buffer, &map, GST_MAP_READ);
                    
                    // Convert GStreamer buffer to OpenCV Mat
                    frame = cv::Mat(height, width, CV_8UC3, (char*)map.data);
                    frame = frame.clone();  // Deep copy
                    
                    gst_buffer_unmap(buffer, &map);
                    gst_sample_unref(sample);
                    frameReceived = true;
                }
            } else {
                // Fallback to OpenCV capture
                frameReceived = cap.read(frame);
            }
            
            if (!frameReceived) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            
            totalFrames++;
            
            // Calculate capture latency
            auto captureEnd = std::chrono::high_resolution_clock::now();
            auto latency = std::chrono::duration_cast<std::chrono::microseconds>(captureEnd - captureStart);
            avgCaptureLatency = (avgCaptureLatency * (totalFrames - 1) + latency.count()) / totalFrames;
            
            // Add to processing queue
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                if (frameQueue.size() > 2) {
                    frameQueue.pop();  // Drop frames to maintain low latency
                }
                frameQueue.push(frame.clone());
            }
            queueCondition.notify_one();
            
            // Frame rate control
            auto frameEnd = std::chrono::high_resolution_clock::now();
            auto frameDuration = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - frameStart);
            int targetFrameTime = 1000000 / targetFps;  // microseconds
            
            if (frameDuration.count() < targetFrameTime) {
                std::this_thread::sleep_for(std::chrono::microseconds(targetFrameTime - frameDuration.count()));
            }
            frameStart = std::chrono::high_resolution_clock::now();
        }
    }

    /**
     * TI accelerator processing thread (C7x DSP operations)
     */
    void acceleratorProcessing() {
        while (!stopProcessing) {
            cv::Mat frame;
            
            // Wait for frames
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                queueCondition.wait(lock, [this] { return !frameQueue.empty() || stopProcessing; });
                
                if (stopProcessing) break;
                if (frameQueue.empty()) continue;
                
                frame = frameQueue.front();
                frameQueue.pop();
            }
            
            auto accelStart = std::chrono::high_resolution_clock::now();
            
            #ifdef TI_TDA4VM_PLATFORM
            if (currentMode >= DSP_ACCELERATED) {
                processWithDSPAcceleration(frame);
            } else 
            #endif
            {
                processWithCPUOptimizations(frame);
            }
            
            acceleratedFrames++;
            
            // Calculate accelerator latency
            auto accelEnd = std::chrono::high_resolution_clock::now();
            auto latency = std::chrono::duration_cast<std::chrono::microseconds>(accelEnd - accelStart);
            avgAcceleratorLatency = (avgAcceleratorLatency * (acceleratedFrames - 1) + latency.count()) / acceleratedFrames;
            
            // Queue for display
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                frameQueue.push(frame);
            }
        }
    }

    #ifdef TI_TDA4VM_PLATFORM
    /**
     * Process frame using C7x DSP acceleration via OpenVX
     */
    void processWithDSPAcceleration(cv::Mat& frame) {
        // Convert OpenCV Mat to OpenVX image
        vx_rectangle_t rect = {0, 0, (vx_uint32)width, (vx_uint32)height};
        vx_imagepatch_addressing_t addr = {
            width * 3, 3, width * 3, height, VX_SCALE_UNITY, VX_SCALE_UNITY, 1, 1
        };
        
        void* ptrs[] = {frame.data};
        vxCopyImagePatch(vxInputImage, &rect, 0, &addr, ptrs, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
        
        // Create processing nodes (these run on DSP)
        vx_node gaussianNode = vxGaussian3x3Node(vxGraph, vxInputImage, vxOutputImage);
        
        // Verify and process graph
        vx_status status = vxVerifyGraph(vxGraph);
        if (status == VX_SUCCESS) {
            vxProcessGraph(vxGraph);
            
            // Copy result back to OpenCV Mat
            vxCopyImagePatch(vxOutputImage, &rect, 0, &addr, ptrs, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
        }
        
        // Clean up nodes
        vxReleaseNode(&gaussianNode);
    }
    #endif

    /**
     * CPU-optimized processing with NEON acceleration
     */
    void processWithCPUOptimizations(cv::Mat& frame) {
        #ifdef __aarch64__
        // NEON-optimized operations
        applyNEONEnhancements(frame);
        #endif
        
        // Standard OpenCV operations optimized for A72 cores
        cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0.5);
    }

    #ifdef __aarch64__
    /**
     * NEON SIMD optimized operations
     */
    void applyNEONEnhancements(cv::Mat& frame) {
        uint8_t* data = frame.data;
        int totalPixels = frame.rows * frame.cols * frame.channels();
        int chunks = totalPixels / 16;
        
        // NEON-optimized brightness adjustment
        uint8x16_t brightness = vdupq_n_u8(5);
        
        for (int i = 0; i < chunks; i++) {
            uint8x16_t pixels = vld1q_u8(data + i * 16);
            pixels = vqaddq_u8(pixels, brightness);
            vst1q_u8(data + i * 16, pixels);
        }
    }
    #endif

    /**
     * Main processing and display thread
     */
    void processFrames() {
        while (!stopProcessing) {
            cv::Mat frame;
            
            // Get processed frame
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                queueCondition.wait(lock, [this] { return !frameQueue.empty() || stopProcessing; });
                
                if (stopProcessing) break;
                if (frameQueue.empty()) continue;
                
                frame = frameQueue.front();
                frameQueue.pop();
            }
            
            auto processStart = std::chrono::high_resolution_clock::now();
            
            processedFrames++;
            
            // Add TDA4VM performance overlay
            addTDA4VMOverlay(frame, processStart);
            
            // Display frame
            cv::imshow("TI TDA4VM Camera Test - BeagleBone AI-64", frame);
            
            // Calculate process latency
            auto processEnd = std::chrono::high_resolution_clock::now();
            auto latency = std::chrono::duration_cast<std::chrono::microseconds>(processEnd - processStart);
            avgProcessLatency = (avgProcessLatency * (processedFrames - 1) + latency.count()) / processedFrames;
            
            // Handle keyboard input
            int key = cv::waitKey(1) & 0xFF;
            handleKeyInput(key, frame);
        }
    }

    /**
     * Add TDA4VM-specific performance overlay
     */
    void addTDA4VMOverlay(cv::Mat& frame, const std::chrono::high_resolution_clock::time_point& processStart) {
        // Performance calculations
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
        double elapsedSeconds = elapsed.count() / 1000.0;
        
        double captureFps = (elapsedSeconds > 0) ? totalFrames / elapsedSeconds : 0.0;
        double processFps = (elapsedSeconds > 0) ? processedFrames / elapsedSeconds : 0.0;
        double accelFps = (elapsedSeconds > 0) ? acceleratedFrames / elapsedSeconds : 0.0;

        // Font settings
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        cv::Scalar green(0, 255, 0);
        cv::Scalar blue(255, 0, 0);
        cv::Scalar red(0, 0, 255);
        int thickness = 1;

        // TDA4VM Performance Metrics
        cv::putText(frame, "=== TI TDA4VM Performance ===", cv::Point(10, 20), font, 0.6, blue, 2);
        
        std::string captureText = "Capture FPS: " + std::to_string(captureFps).substr(0, 5);
        cv::putText(frame, captureText, cv::Point(10, 45), font, fontScale, green, thickness);

        std::string processText = "Process FPS: " + std::to_string(processFps).substr(0, 5);
        cv::putText(frame, processText, cv::Point(10, 65), font, fontScale, green, thickness);

        std::string accelText = "Accel FPS: " + std::to_string(accelFps).substr(0, 5);
        cv::putText(frame, accelText, cv::Point(10, 85), font, fontScale, green, thickness);

        // Latency information
        cv::putText(frame, "=== Latencies (μs) ===", cv::Point(200, 20), font, 0.6, blue, 2);
        cv::putText(frame, "Capture: " + std::to_string((int)avgCaptureLatency), cv::Point(200, 45), font, fontScale, green, thickness);
        cv::putText(frame, "Process: " + std::to_string((int)avgProcessLatency), cv::Point(200, 65), font, fontScale, green, thickness);
        cv::putText(frame, "Accel: " + std::to_string((int)avgAcceleratorLatency), cv::Point(200, 85), font, fontScale, green, thickness);

        // Processing mode
        std::string modeText = "Mode: ";
        cv::Scalar modeColor = green;
        switch (currentMode) {
            case CPU_ONLY: modeText += "CPU (A72)"; modeColor = red; break;
            case DSP_ACCELERATED: modeText += "DSP (C7x)"; modeColor = blue; break;
            case FULL_HARDWARE: modeText += "Full HW"; modeColor = green; break;
        }
        cv::putText(frame, modeText, cv::Point(10, 110), font, fontScale, modeColor, thickness);

        // Pipeline type
        std::string pipeText = "Pipeline: " + std::string(useGStreamerPipeline ? "GStreamer+ISP" : "OpenCV+V4L2");
        cv::putText(frame, pipeText, cv::Point(10, 130), font, fontScale, green, thickness);

        // Queue status
        std::string queueText = "Queue: " + std::to_string(frameQueue.size());
        cv::putText(frame, queueText, cv::Point(10, 150), font, fontScale, green, thickness);

        // Draw center crosshair
        int centerX = frame.cols / 2;
        int centerY = frame.rows / 2;
        cv::drawMarker(frame, cv::Point(centerX, centerY), cv::Scalar(255, 255, 0),
                      cv::MARKER_CROSS, 30, 2);
    }

    /**
     * Handle keyboard input
     */
    void handleKeyInput(int key, const cv::Mat& frame) {
        switch (key) {
            case 'q':
            case 'Q':
                std::cout << "Quitting TDA4VM camera test..." << std::endl;
                stopProcessing = true;
                break;
            case 's':
            case 'S':
                saveFrame(frame);
                break;
            case 'i':
            case 'I':
                printTDA4VMInfo();
                break;
            case 'm':
            case 'M':
                cycleModes();
                break;
            case 'p':
            case 'P':
                togglePipeline();
                break;
        }
    }

    /**
     * Cycle through processing modes
     */
    void cycleModes() {
        currentMode = static_cast<ProcessingMode>((currentMode + 1) % 3);
        std::string modeStr;
        switch (currentMode) {
            case CPU_ONLY: modeStr = "CPU Only (A72)"; break;
            case DSP_ACCELERATED: modeStr = "DSP Accelerated (C7x)"; break;
            case FULL_HARDWARE: modeStr = "Full Hardware"; break;
        }
        std::cout << "Switched to mode: " << modeStr << std::endl;
    }

    /**
     * Toggle between GStreamer and OpenCV pipeline
     */
    void togglePipeline() {
        // This would require reinitializing the camera
        std::cout << "Pipeline toggle requested (requires restart)" << std::endl;
    }

    /**
     * Run the TDA4VM optimized camera test
     */
    bool runTest() {
        if (!initializeCamera()) {
            return false;
        }

        std::cout << "\nTI TDA4VM Camera Test Running..." << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  'q' - Quit" << std::endl;
        std::cout << "  's' - Save frame" << std::endl;
        std::cout << "  'i' - System info" << std::endl;
        std::cout << "  'm' - Cycle processing modes" << std::endl;
        std::cout << "  'p' - Toggle pipeline" << std::endl;

        startTime = std::chrono::high_resolution_clock::now();
        stopProcessing = false;

        // Start processing threads
        captureThread = std::thread(&TDA4VMCameraTest::captureFrames, this);
        acceleratorThread = std::thread(&TDA4VMCameraTest::acceleratorProcessing, this);
        processThread = std::thread(&TDA4VMCameraTest::processFrames, this);

        // Wait for completion
        processThread.join();
        stopProcessing = true;
        queueCondition.notify_all();
        acceleratorThread.join();
        captureThread.join();

        printFinalStats();
        cleanup();
        return true;
    }

    /**
     * Save frame with TDA4VM prefix
     */
    void saveFrame(const cv::Mat& frame) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        
        std::string filename = "tda4vm_test_" + ss.str() + ".jpg";
        if (cv::imwrite(filename, frame)) {
            std::cout << "Frame saved as: " << filename << std::endl;
        }
    }

    /**
     * Print TDA4VM system information
     */
    void printTDA4VMInfo() {
        std::cout << "\n=== TI TDA4VM System Information ===" << std::endl;
        std::cout << "SoC: TI TDA4VM (Jacinto 7 family)" << std::endl;
        std::cout << "Processing Mode: ";
        switch (currentMode) {
            case CPU_ONLY: std::cout << "CPU Only (A72 cores)"; break;
            case DSP_ACCELERATED: std::cout << "DSP Accelerated (C7x cores)"; break;
            case FULL_HARDWARE: std::cout << "Full Hardware Pipeline"; break;
        }
        std::cout << std::endl;
        std::cout << "Video Pipeline: " << (useGStreamerPipeline ? "GStreamer + TI ISP" : "OpenCV + V4L2") << std::endl;
        std::cout << "Resolution: " << width << "x" << height << "@" << targetFps << "fps" << std::endl;
        std::cout << "===================================\n" << std::endl;
    }

    /**
     * Print final performance statistics
     */
    void printFinalStats() {
        auto endTime = std::chrono::high_resolution_clock::now();
        auto totalTime = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
        
        std::cout << "\n=== TI TDA4VM Final Performance Stats ===" << std::endl;
        std::cout << "Runtime: " << totalTime.count() << "s" << std::endl;
        std::cout << "Frames Captured: " << totalFrames << std::endl;
        std::cout << "Frames Processed: " << processedFrames << std::endl;
        std::cout << "Frames Accelerated: " << acceleratedFrames << std::endl;
        std::cout << "Avg Capture FPS: " << (totalTime.count() > 0 ? totalFrames / totalTime.count() : 0) << std::endl;
        std::cout << "Avg Process FPS: " << (totalTime.count() > 0 ? processedFrames / totalTime.count() : 0) << std::endl;
        std::cout << "Avg Capture Latency: " << avgCaptureLatency << "μs" << std::endl;
        std::cout << "Avg Process Latency: " << avgProcessLatency << "μs" << std::endl;
        std::cout << "Avg Accelerator Latency: " << avgAcceleratorLatency << "μs" << std::endl;
        std::cout << "=========================================" << std::endl;
    }

    /**
     * Cleanup resources
     */
    void cleanup() {
        stopProcessing = true;
        
        if (useGStreamerPipeline && pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            if (appsink) {
                gst_object_unref(appsink);
            }
        }
        
        if (cap.isOpened()) {
            cap.release();
        }
        
        #ifdef TI_TDA4VM_PLATFORM
        if (vxContext) {
            if (vxInputImage) vxReleaseImage(&vxInputImage);
            if (vxOutputImage) vxReleaseImage(&vxOutputImage);
            if (vxGraph) vxReleaseGraph(&vxGraph);
            vxReleaseContext(&vxContext);
            tivxHostDeInit();
            tivxDeInit();
        }
        #endif
        
        cv::destroyAllWindows();
        gst_deinit();
        std::cout << "TDA4VM resources cleaned up" << std::endl;
    }

    /**
     * Destructor
     */
    ~TDA4VMCameraTest() {
        cleanup();
    }
};

/**
 * Main function
 */
int main() {
    std::cout << "=========================================" << std::endl;
    std::cout << "TI TDA4VM Camera Test - BeagleBone AI-64" << std::endl;
    std::cout << "Hardware Accelerated Vision Pipeline" << std::endl;
    std::cout << "Jacinto 7 SoC with C7x DSP + MMA + ISP" << std::endl;
    std::cout << "=========================================" << std::endl;

    // Try different camera indices and resolutions
    std::vector<std::tuple<int, int, int, int>> configs = {
        {0, 1280, 720, 30},  // HD
        {0, 640, 480, 60},   // VGA high FPS
        {1, 640, 480, 30},   // Alternative camera
        {2, 640, 480, 30}    // USB camera
    };
    
    bool success = false;

    for (const auto& config : configs) {
        int camIndex = std::get<0>(config);
        int width = std::get<1>(config);
        int height = std::get<2>(config);
        int fps = std::get<3>(config);
        
        std::cout << "\nTrying camera " << camIndex 
                  << " at " << width << "x" << height << "@" << fps << "fps..." << std::endl;
        
        TDA4VMCameraTest cameraTest(camIndex, width, height, fps);
        
        if (cameraTest.runTest()) {
            success = true;
            break;
        } else {
            std::cout << "Configuration failed, trying next..." << std::endl;
        }
    }

    if (!success) {
        std::cout << "Error: No working camera configuration found!" << std::endl;
        std::cout << "Make sure:" << std::endl;
        std::cout << "1. Camera is connected and /dev/videoX exists" << std::endl;
        std::cout << "2. TI Processor SDK is installed" << std::endl;
        std::cout << "3. EdgeAI components are available" << std::endl;
        return 1;
    }

    std::cout << "TI TDA4VM camera test completed successfully!" << std::endl;
    return 0;
}
