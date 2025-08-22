#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>  // Added for fabs
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// WiFi Configuration
#define WIFI_SSID      "delta_virus"
#define WIFI_PASS      "66380115"
#define WIFI_MAXIMUM_RETRY  5

// Motor Pin Definitions for TB6612FNG
#define LEFT_IN1       16  // AIN1: Left motor
#define LEFT_IN2       17  // AIN2: Left motor
#define RIGHT_IN1      18  // BIN1: Right motor
#define RIGHT_IN2      19  // BIN2: Right motor
#define PWM_A          21  // PWMA: PWM for left motor
#define PWM_B          22  // PWMB: PWM for right motor
#define STBY_PIN       23  // STBY: Set high to enable driver

// PWM Configuration
#define PWM_FREQ_HZ    1000  // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_10_BIT  // 10-bit PWM resolution
#define MAX_PWM_DUTY   1023  // Max duty cycle for 10-bit resolution
#define WHEEL_BASE     0.2   // Wheel base distance in meters (adjust as needed)
#define MAX_LINEAR_VEL 0.5   // Max linear velocity (m/s)
#define MAX_ANGULAR_VEL 2.0  // Max angular velocity (rad/s)

// FreeRTOS event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "MOTOR_HTTP";
static int s_retry_num = 0;

// Motor control structure to store velocities
typedef struct {
    float linear_vel;  // Linear velocity (m/s)
    float angular_vel; // Angular velocity (rad/s)
} cmd_vel_t;

// Global command velocity
static cmd_vel_t cmd_vel = {0.0, 0.0};

// Function prototypes
void motor_stop(void);  // Added prototype to resolve implicit declaration

// PWM channel configuration
static void pwm_init(void) {
    // Configure PWM timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    // Configure PWM channels
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = PWM_A,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = PWM_B,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        }
    };
    ledc_channel_config(&ledc_channel[0]);
    ledc_channel_config(&ledc_channel[1]);
}

// Motor initialization
void motor_init(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LEFT_IN1) | (1ULL << LEFT_IN2) |
                           (1ULL << RIGHT_IN1) | (1ULL << RIGHT_IN2) |
                           (1ULL << STBY_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Initialize PWM
    pwm_init();

    // Enable motor driver
    gpio_set_level(STBY_PIN, 1);

    // Initially stop motors
    motor_stop();
}

// Set motor speeds based on linear and angular velocities
void set_motor_speeds(float linear_vel, float angular_vel) {
    // Calculate left and right wheel velocities
    float left_vel = linear_vel - (angular_vel * WHEEL_BASE / 2.0);
    float right_vel = linear_vel + (angular_vel * WHEEL_BASE / 2.0);

    // Normalize to PWM duty cycle (0 to MAX_PWM_DUTY)
    int left_duty = (int)(fabs(left_vel) / MAX_LINEAR_VEL * MAX_PWM_DUTY);
    int right_duty = (int)(fabs(right_vel) / MAX_LINEAR_VEL * MAX_PWM_DUTY);

    // Clamp duty cycles
    left_duty = left_duty > MAX_PWM_DUTY ? MAX_PWM_DUTY : left_duty;
    right_duty = right_duty > MAX_PWM_DUTY ? MAX_PWM_DUTY : right_duty;

    // Set direction
    if (left_vel >= 0) {
        gpio_set_level(LEFT_IN1, 1);
        gpio_set_level(LEFT_IN2, 0);
    } else {
        gpio_set_level(LEFT_IN1, 0);
        gpio_set_level(LEFT_IN2, 1);
    }

    if (right_vel >= 0) {
        gpio_set_level(RIGHT_IN1, 1);
        gpio_set_level(RIGHT_IN2, 0);
    } else {
        gpio_set_level(RIGHT_IN1, 0);
        gpio_set_level(RIGHT_IN2, 1);
    }

    // Update PWM duties
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, left_duty);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, right_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

    ESP_LOGI(TAG, "Set velocities: linear=%.2f m/s, angular=%.2f rad/s, left_duty=%d, right_duty=%d",
             linear_vel, angular_vel, left_duty, right_duty);
}

// Motor control functions
void motor_forward(void) {
    cmd_vel.linear_vel = MAX_LINEAR_VEL * 0.5; // Default 50% speed
    cmd_vel.angular_vel = 0.0;
    set_motor_speeds(cmd_vel.linear_vel, cmd_vel.angular_vel);
}

void motor_backward(void) {
    cmd_vel.linear_vel = -MAX_LINEAR_VEL * 0.5;
    cmd_vel.angular_vel = 0.0;
    set_motor_speeds(cmd_vel.linear_vel, cmd_vel.angular_vel);
}

void motor_left(void) {
    cmd_vel.linear_vel = 0.0;
    cmd_vel.angular_vel = MAX_ANGULAR_VEL * 0.5;
    set_motor_speeds(cmd_vel.linear_vel, cmd_vel.angular_vel);
}

void motor_right(void) {
    cmd_vel.linear_vel = 0.0;
    cmd_vel.angular_vel = -MAX_ANGULAR_VEL * 0.5;
    set_motor_speeds(cmd_vel.linear_vel, cmd_vel.angular_vel);
}

void motor_stop(void) {
    cmd_vel.linear_vel = 0.0;
    cmd_vel.angular_vel = 0.0;
    gpio_set_level(LEFT_IN1, 1);
    gpio_set_level(LEFT_IN2, 1);
    gpio_set_level(RIGHT_IN1, 1);
    gpio_set_level(RIGHT_IN2, 1);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ESP_LOGI(TAG, "Motors stopped");
}

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// HTTP handler for velocity command
esp_err_t cmd_vel_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read content");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    // Parse linear and angular velocities
    float linear_vel, angular_vel;
    if (sscanf(buf, "linear=%f&angular=%f", &linear_vel, &angular_vel) != 2) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid format");
        return ESP_FAIL;
    }

    // Clamp velocities
    linear_vel = linear_vel > MAX_LINEAR_VEL ? MAX_LINEAR_VEL : linear_vel;
    linear_vel = linear_vel < -MAX_LINEAR_VEL ? -MAX_LINEAR_VEL : linear_vel;
    angular_vel = angular_vel > MAX_ANGULAR_VEL ? MAX_ANGULAR_VEL : angular_vel;
    angular_vel = angular_vel < -MAX_ANGULAR_VEL ? -MAX_ANGULAR_VEL : angular_vel;

    cmd_vel.linear_vel = linear_vel;
    cmd_vel.angular_vel = angular_vel;
    set_motor_speeds(linear_vel, angular_vel);

    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP handlers for motor control
esp_err_t motor_forward_handler(httpd_req_t *req) {
    motor_forward();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t motor_backward_handler(httpd_req_t *req) {
    motor_backward();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t motor_left_handler(httpd_req_t *req) {
    motor_left();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t motor_right_handler(httpd_req_t *req) {
    motor_right();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t motor_stop_handler(httpd_req_t *req) {
    motor_stop();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Updated HTML page with velocity control sliders
static const char* html_page =
"<!DOCTYPE html>"
"<html>"
"<head>"
"    <title>Motor Control</title>"
"    <meta name='viewport' content='width=device-width, initial-scale=1'>"
"    <style>"
"        body { "
"            font-family: Arial, sans-serif; "
"            text-align: center; "
"            margin: 0; "
"            padding: 20px; "
"            background-color: #f5f5f5; "
"            color: #333; "
"        }"
"        .container { "
"            max-width: 400px; "
"            margin: 0 auto; "
"            background-color: white; "
"            border-radius: 8px; "
"            box-shadow: 0 2px 10px rgba(0,0,0,0.1); "
"            padding: 30px; "
"        }"
"        h1 { "
"            color: #000; "
"            margin-bottom: 30px; "
"            font-size: 24px; "
"        }"
"        .status { "
"            background-color: #000; "
"            color: white; "
"            padding: 12px; "
"            border-radius: 4px; "
"            margin-bottom: 25px; "
"            font-size: 14px; "
"        }"
"        .control-grid { "
"            display: grid; "
"            grid-template-columns: 1fr 1fr 1fr; "
"            gap: 12px; "
"            margin: 25px 0; "
"        }"
"        button { "
"            font-size: 16px; "
"            padding: 20px 15px; "
"            border: 2px solid #000; "
"            border-radius: 4px; "
"            cursor: pointer; "
"            background-color: white; "
"            color: #000; "
"            transition: all 0.2s; "
"            font-weight: 500; "
"        }"
"        button:hover { "
"            background-color: #000; "
"            color: white; "
"        }"
"        button:active { "
"            background-color: #333; "
"        }"
"        .forward { grid-column: 2; }"
"        .left { grid-column: 1; grid-row: 2; }"
"        .stop { "
"            grid-column: 2; "
"            grid-row: 2; "
"            background-color: #000; "
"            color: white; "
"        }"
"        .stop:hover { "
"            background-color: #333; "
"        }"
"        .right { grid-column: 3; grid-row: 2; }"
"        .backward { grid-column: 2; grid-row: 3; }"
"        .instructions { "
"            color: #666; "
"            font-size: 12px; "
"            margin-bottom: 20px; "
"        }"
"        .slider-container { "
"            margin: 20px 0; "
"        }"
"        input[type='range'] { "
"            width: 100%; "
"            margin: 10px 0; "
"        }"
"        label { "
"            display: block; "
"            font-size: 14px; "
"            margin-bottom: 5px; "
"        }"
"    </style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>Motor Control</h1>"
"        <div class='status' id='status'>Ready</div>"
"        <div class='instructions'>Use sliders for velocity control or buttons for preset movements</div>"
"        <div class='slider-container'>"
"            <label>Linear Velocity (m/s): <span id='linearValue'>0.0</span></label>"
"            <input type='range' id='linearSlider' min='-0.5' max='0.5' step='0.01' value='0'>"
"            <label>Angular Velocity (rad/s): <span id='angularValue'>0.0</span></label>"
"            <input type='range' id='angularSlider' min='-2' max='2' step='0.01' value='0'>"
"        </div>"
"        <div class='control-grid'>"
"            <button class='forward' onmousedown='sendCommand(\"forward\")' onmouseup='sendCommand(\"stop\")' ontouchstart='sendCommand(\"forward\")' ontouchend='sendCommand(\"stop\")'>Forward</button>"
"            <button class='left' onmousedown='sendCommand(\"left\")' onmouseup='sendCommand(\"stop\")' ontouchstart='sendCommand(\"left\")' ontouchend='sendCommand(\"stop\")'>Left</button>"
"            <button class='stop' onclick='sendCommand(\"stop\")'>STOP</button>"
"            <button class='right' onmousedown='sendCommand(\"right\")' onmouseup='sendCommand(\"stop\")' ontouchstart='sendCommand(\"right\")' ontouchend='sendCommand(\"stop\")'>Right</button>"
"            <button class='backward' onmousedown='sendCommand(\"backward\")' onmouseup='sendCommand(\"stop\")' ontouchstart='sendCommand(\"backward\")' ontouchend='sendCommand(\"stop\")'>Backward</button>"
"        </div>"
"    </div>"
"    <script>"
"        let status = document.getElementById('status');"
"        let linearSlider = document.getElementById('linearSlider');"
"        let angularSlider = document.getElementById('angularSlider');"
"        let linearValue = document.getElementById('linearValue');"
"        let angularValue = document.getElementById('angularValue');"
"        let currentCommand = '';"
"        function sendCommand(command) {"
"            if (currentCommand === command) return;"
"            currentCommand = command;"
"            status.textContent = command.charAt(0).toUpperCase() + command.slice(1);"
"            fetch('/' + command)"
"                .then(response => response.text())"
"                .then(data => {"
"                    if (command === 'stop') {"
"                        status.textContent = 'Stopped';"
"                        currentCommand = '';"
"                        linearSlider.value = 0;"
"                        angularSlider.value = 0;"
"                        linearValue.textContent = '0.0';"
"                        angularValue.textContent = '0.0';"
"                    }"
"                })"
"                .catch(error => {"
"                    status.textContent = 'Error';"
"                });"
"        }"
"        function sendVelocity() {"
"            let linear = parseFloat(linearSlider.value);"
"            let angular = parseFloat(angularSlider.value);"
"            linearValue.textContent = linear.toFixed(2);"
"            angularValue.textContent = angular.toFixed(2);"
"            status.textContent = `Linear: ${linear.toFixed(2)} m/s, Angular: ${angular.toFixed(2)} rad/s`;"
"            fetch('/cmd_vel', {"
"                method: 'POST',"
"                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },"
"                body: `linear=${linear}&angular=${angular}`"
"            })"
"                .catch(error => {"
"                    status.textContent = 'Error';"
"                });"
"        }"
"        linearSlider.addEventListener('input', sendVelocity);"
"        angularSlider.addEventListener('input', sendVelocity);"
"        document.addEventListener('contextmenu', function(e) {"
"            e.preventDefault();"
"        });"
"    </script>"
"</body>"
"</html>";

// HTTP handler for main page
esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// HTTP server configuration
httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");

        // Root handler
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);

        // Motor control handlers
        httpd_uri_t forward_uri = {
            .uri       = "/forward",
            .method    = HTTP_GET,
            .handler   = motor_forward_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &forward_uri);

        httpd_uri_t backward_uri = {
            .uri       = "/backward",
            .method    = HTTP_GET,
            .handler   = motor_backward_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &backward_uri);

        httpd_uri_t left_uri = {
            .uri       = "/left",
            .method    = HTTP_GET,
            .handler   = motor_left_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &left_uri);

        httpd_uri_t right_uri = {
            .uri       = "/right",
            .method    = HTTP_GET,
            .handler   = motor_right_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &right_uri);

        httpd_uri_t stop_uri = {
            .uri       = "/stop",
            .method    = HTTP_GET,
            .handler   = motor_stop_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stop_uri);

        httpd_uri_t cmd_vel_uri = {
            .uri       = "/cmd_vel",
            .method    = HTTP_POST,
            .handler   = cmd_vel_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cmd_vel_uri);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize motor pins and PWM
    motor_init();

    // Connect to WiFi
    wifi_init_sta();

    // Start web server
    start_webserver();

    ESP_LOGI(TAG, "Motor control server started. Connect to the IP address shown above.");
}
