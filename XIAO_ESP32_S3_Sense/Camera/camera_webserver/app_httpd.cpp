/**
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * XIAO ESP32-S3 SENSE | Asynchronous HTTP Streamer Implementation
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * FILE: app_httpd.cpp
 * DESCRIPTION:
 * This file handles the low-level HTTP socket management. It routes 
 * requests from the browser to the camera sensor and serves the 
 * compressed assets from 'camera_index.h'.
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * @file app_httpd.cpp
 * @author Gemini Thought Partner
 * @brief High-availability Async Web Server for XIAO ESP32-S3.
 * * Features: 
 * - Dual-port architecture (80 for UI/Control, 81 for Stream)
 * - Proactive memory reclamation
 * - CORS-compliant headers for cross-origin security
 * * Summary:
 *      Thread Safety: By using vTaskDelete(NULL) at the end of the server setup task, we ensure that the setup memory is reclaimed 
 *      while the web servers continue to run in the background.
 *      Memory Hygiene: Every time esp_camera_fb_get() is called, we explicitly call esp_camera_fb_return(). 
 *      Failure to do this is the #1 reason for ESP32-S3 reboots.
 *      Non-Blocking Logic: The use of vTaskDelay(1) inside the infinite stream loop is what allows the "Stop Stream" command 
 *      to actually get through to the CPU. Without it, the stream would "hog" the core, making the device unresponsive to web commands.
 */

#include "Arduino.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"

/* --- System State --- */
bool is_streaming = false; 
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

/* --- MJPEG Protocol Constants --- */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/**
 * @brief Handler for single frame capture (Still Image).
 * Implements strict error checking for DMA buffer acquisition.
 */
static esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("\033[1;31m[ERROR]\033[0m Camera capture failed: Buffer Timeout");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("\033[1;34m[DATA]\033[0m Still Captured: %uB (%ums)\n", (uint32_t)(fb->len), (uint32_t)((fr_end - fr_start)/1000));
    
    esp_camera_fb_return(fb);
    return res;
}

/**
 * @brief Persistent MJPEG Stream Handler.
 * Designed to yield to the Wi-Fi stack periodically to prevent socket closure.
 */
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    if(res != ESP_OK) return res;

    is_streaming = true;
    Serial.println("\033[1;32m[INFO]\033[0m Client connected to MJPEG Stream");

    while(true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("\033[1;33m[WARN]\033[0m Stream Buffer Timeout");
            res = ESP_FAIL;
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if(res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        if(fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }

        if(res != ESP_OK) {
            break; 
        }

        // Essential yield for system stability
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    is_streaming = false;
    Serial.println("\033[1;32m[INFO]\033[0m MJPEG Stream Session Terminated");
    return res;
}

/**
 * @brief System Status Provider.
 * Delivers JSON-encoded sensor data to the Web UI.
 */
static esp_err_t status_handler(httpd_req_t *req) {
    static char json_response[128];
    sensor_t *s = esp_camera_sensor_get();
    
    if (!s) {
        return httpd_resp_send_500(req);
    }

    char *p = json_response;
    *p++ = '{';
    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u", s->status.quality);
    *p++ = '}';
    *p++ = 0;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    return httpd_resp_send(req, json_response, strlen(json_response));
}

/**
 * @brief Index Handler. Serves the Gzipped HTML from memory.
 */
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

/**
 * @brief Server Configuration Task.
 * Initializes Port 80 and 81 instances with optimized stack sizes.
 */
void startCameraServerTask(void *pvParameters) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 10;

    // UI Handlers (Port 80)
    httpd_uri_t index_uri   = { "/",        HTTP_GET, index_handler,   NULL };
    httpd_uri_t status_uri  = { "/status",   HTTP_GET, status_handler,  NULL };
    httpd_uri_t capture_uri = { "/capture",  HTTP_GET, capture_handler, NULL };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        Serial.println("\033[1;32m[OK]\033[0m Port 80 (Control) Initialized");
    }

    // High-Bandwidth Stream Handler (Port 81)
    httpd_config_t s_config = HTTPD_DEFAULT_CONFIG();
    s_config.server_port = 81;
    s_config.ctrl_port = 32769; // Ensures independent control socket
    
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };

    if (httpd_start(&stream_httpd, &s_config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("\033[1;32m[OK]\033[0m Port 81 (Stream) Initialized");
    }

    vTaskDelete(NULL);
}

/**
 * @brief Public Entry Point for Camera Server.
 * Spawns the server task on Core 0 to isolate networking from user logic.
 */
void startCameraServer() {
    xTaskCreatePinnedToCore(startCameraServerTask, "WebSrvTask", 4096, NULL, 5, NULL, 0);
}