#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// WiFi Configuration
#define WIFI_SSID      "delta_virus"
#define WIFI_PASS      "66380115"
#define WIFI_MAXIMUM_RETRY  5

// Motor Pin Definitions
#define MOTOR1_PIN1    16  // Left motor
#define MOTOR1_PIN2    17  // Left motor
#define MOTOR2_PIN1    18  // Right motor
#define MOTOR2_PIN2    19  // Right motor

// FreeRTOS event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "MOTOR_HTTP";
static int s_retry_num = 0;

// Motor control functions
void motor_init(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR1_PIN1) | (1ULL << MOTOR1_PIN2) |
                          (1ULL << MOTOR2_PIN1) | (1ULL << MOTOR2_PIN2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Initially stop all motors
    gpio_set_level(MOTOR1_PIN1, 0);
    gpio_set_level(MOTOR1_PIN2, 0);
    gpio_set_level(MOTOR2_PIN1, 0);
    gpio_set_level(MOTOR2_PIN2, 0);
}

void motor_forward(void) {
    ESP_LOGI(TAG, "Moving Forward");
    gpio_set_level(MOTOR1_PIN1, 1);
    gpio_set_level(MOTOR1_PIN2, 0);
    gpio_set_level(MOTOR2_PIN1, 1);
    gpio_set_level(MOTOR2_PIN2, 0);
}

void motor_backward(void) {
    ESP_LOGI(TAG, "Moving Backward");
    gpio_set_level(MOTOR1_PIN1, 0);
    gpio_set_level(MOTOR1_PIN2, 1);
    gpio_set_level(MOTOR2_PIN1, 0);
    gpio_set_level(MOTOR2_PIN2, 1);
}

void motor_left(void) {
    ESP_LOGI(TAG, "Turning Left");
    gpio_set_level(MOTOR1_PIN1, 0);  // Left motor backward
    gpio_set_level(MOTOR1_PIN2, 1);
    gpio_set_level(MOTOR2_PIN1, 1);  // Right motor forward
    gpio_set_level(MOTOR2_PIN2, 0);
}

void motor_right(void) {
    ESP_LOGI(TAG, "Turning Right");
    gpio_set_level(MOTOR1_PIN1, 1);  // Left motor forward
    gpio_set_level(MOTOR1_PIN2, 0);
    gpio_set_level(MOTOR2_PIN1, 0);  // Right motor backward
    gpio_set_level(MOTOR2_PIN2, 1);
}

void motor_stop(void) {
    ESP_LOGI(TAG, "Stopping Motors");
    gpio_set_level(MOTOR1_PIN1, 0);
    gpio_set_level(MOTOR1_PIN2, 0);
    gpio_set_level(MOTOR2_PIN1, 0);
    gpio_set_level(MOTOR2_PIN2, 0);
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
        ESP_LOGI(TAG,"connect to the AP fail");
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

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

// HTML page with simplified monochrome motor control interface
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
"    </style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>Motor Control</h1>"
"        <div class='status' id='status'>Ready</div>"
"        <div class='instructions'>Hold buttons to move, release to stop</div>"
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
"                    }"
"                })"
"                .catch(error => {"
"                    status.textContent = 'Error';"
"                });"
"        }"
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

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize motor pins
    motor_init();

    // Connect to WiFi
    wifi_init_sta();

    // Start web server
    start_webserver();

    ESP_LOGI(TAG, "Motor control server started. Connect to the IP address shown above.");
}
