// ============================================================
// ESP32-CAM AI THINKER
// SCREENSHOT NODE ONLY - NO LIVE STREAM / NO ESP-NOW
//
// Responsibilities
//   - Connect to ESP32_Turbo AP as STA
//   - DHCP IP
//   - Report IP/state to MAIN over UART
//   - Keep Wi-Fi awake
//   - Provide one JPEG frame at /capture
//   - Provide status at /status
//   - UART CAM_ON / CAM_OFF
//   - UART heartbeat every 1 second
//
// UART
//   MAIN GPIO32 TX -> CAM GPIO3 RX
//   MAIN GPIO33 RX <- CAM GPIO1 TX
//   MAIN GND       <-> CAM GND
//
// Laptop flow
//   MAIN /camera/status -> camera IP
//   LAPTOP GET http://CAMERA_IP/capture
//   LAPTOP Edge Impulse Linux x86 model.eim
//   LAPTOP GET http://192.168.4.1/detection?... 
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_camera.h>
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// -------------------- Wi-Fi --------------------
const char* AP_SSID     = "ESP32_Turbo";
const char* AP_PASSWORD = "123456789";
#define WIFI_CHANNEL 6

// -------------------- UART --------------------
#define CAMERA_UART_RX   3
#define CAMERA_UART_TX   1
#define CAMERA_UART_BAUD 115200

// -------------------- AI Thinker camera pins --------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4

// -------------------- Camera --------------------
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA
#define CAMERA_JPEG_QUALITY 12
#define CAMERA_FB_COUNT 2

volatile bool cameraEnabled = false;
volatile bool flashEnabled = false;
SemaphoreHandle_t cameraMutex = nullptr;
httpd_handle_t camera_httpd = nullptr;

unsigned long lastHeartbeat = 0;
unsigned long lastWiFiCheck = 0;
const unsigned long HEARTBEAT_MS = 1000;
const unsigned long WIFI_CHECK_MS = 1000;

// -------------------- UART protocol --------------------
void sendUART(const char* msg) {
  if (msg && msg[0]) Serial.println(msg);
}

void applyFlash(bool on) {
  flashEnabled = on;
  digitalWrite(FLASH_LED_PIN, flashEnabled ? HIGH : LOW);
}

void sendState() {
  sendUART(cameraEnabled ? "CAM_STATE:ON" : "CAM_STATE:OFF");
  sendUART(flashEnabled ? "CAM_FLASH:ON" : "CAM_FLASH:OFF");
  String msg = String("CAM_IP:") + WiFi.localIP().toString();
  Serial.println(msg);
}

void processUART() {
  static char buffer[48];
  static size_t index = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (index == 0) continue;
      buffer[index] = '\0';

      if (strcmp(buffer, "CAM_ON") == 0) {
        cameraEnabled = true;
        sendState();
      }
      else if (strcmp(buffer, "CAM_OFF") == 0) {
        cameraEnabled = false;
        sendUART("CAM_STATE:OFF");
      }
      else if (strcmp(buffer, "FLASH_ON") == 0) {
        applyFlash(true);
        sendUART("CAM_FLASH:ON");
      }
      else if (strcmp(buffer, "FLASH_OFF") == 0) {
        applyFlash(false);
        sendUART("CAM_FLASH:OFF");
      }

      index = 0;
    }
    else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
    else {
      index = 0;
    }
  }
}

void configureWiFi() {
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);
}

bool connectToMainAP() {
  WiFi.mode(WIFI_STA);
  configureWiFi();
  WiFi.begin(AP_SSID, AP_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(150);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI_FAILED");
    return false;
  }

  uint8_t channel = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&channel, &second);

  Serial.println("CAMERA_WIFI_CONNECTED");
  Serial.print("CAMERA_IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("CHANNEL: ");
  Serial.println(channel);
  return true;
}

// -------------------- HTTP --------------------
static esp_err_t root_handler(httpd_req_t* req) {
  const char* body =
    "ESP32-CAM SCREENSHOT NODE\n"
    "GET /capture\n"
    "GET /status\n";
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t* req) {
  char json[192];
  snprintf(
    json, sizeof(json),
    "{\"camera\":\"%s\",\"ip\":\"%s\",\"width\":320,\"height\":240}",
    cameraEnabled ? "ON" : "OFF",
    WiFi.localIP().toString().c_str()
  );
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t* req) {
  if (!cameraEnabled) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Camera OFF", HTTPD_RESP_USE_STRLEN);
  }

  if (!cameraMutex || xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(250)) != pdTRUE) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Camera busy", HTTPD_RESP_USE_STRLEN);
  }

  // Retry a few times so a temporary frame-buffer failure does not
  // force the laptop inference loop to stop.
  camera_fb_t* fb = nullptr;
  const uint8_t CAPTURE_RETRIES = 3;

  for (uint8_t attempt = 0; attempt < CAPTURE_RETRIES; ++attempt) {
    fb = esp_camera_fb_get();
    if (fb) break;
    delay(8);
  }

  if (!fb) {
    // Refresh the camera driver after repeated capture failure.
    esp_camera_deinit();
    delay(30);
    bool recovered = initCamera();

    xSemaphoreGive(cameraMutex);
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(
      req,
      recovered ? "Capture failed - camera refreshed" : "Capture failed - camera refresh failed",
      HTTPD_RESP_USE_STRLEN
    );
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");

  esp_err_t result = httpd_resp_send(
    req,
    (const char*)fb->buf,
    fb->len
  );

  esp_camera_fb_return(fb);
  xSemaphoreGive(cameraMutex);
  return result;
}

void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_open_sockets = 4;
  config.max_uri_handlers = 4;
  config.stack_size = 8192;

  httpd_uri_t rootUri = {};
  rootUri.uri = "/";
  rootUri.method = HTTP_GET;
  rootUri.handler = root_handler;

  httpd_uri_t statusUri = {};
  statusUri.uri = "/status";
  statusUri.method = HTTP_GET;
  statusUri.handler = status_handler;

  httpd_uri_t captureUri = {};
  captureUri.uri = "/capture";
  captureUri.method = HTTP_GET;
  captureUri.handler = capture_handler;

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &rootUri);
    httpd_register_uri_handler(camera_httpd, &statusUri);
    httpd_register_uri_handler(camera_httpd, &captureUri);

    Serial.println("CAMERA HTTP SERVER READY");
    Serial.print("CAPTURE: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/capture");
  }
  else {
    Serial.println("CAMERA HTTP SERVER FAILED");
  }
}

bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;
    config.fb_count = CAMERA_FB_COUNT;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  }
  else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 14;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("CAMERA_INIT_FAILED: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, CAMERA_JPEG_QUALITY);
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
  }

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashEnabled ? HIGH : LOW);

  Serial.println("CAMERA_READY: QVGA 320x240");
  return true;
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("WIFI_RECONNECTING");
  WiFi.disconnect(false, false);
  delay(100);

  if (connectToMainAP()) {
    sendState();
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // UART0 on the normal programming pins.
  // Disconnect MAIN TX/RX during upload.
  Serial.begin(
    CAMERA_UART_BAUD,
    SERIAL_8N1,
    CAMERA_UART_RX,
    CAMERA_UART_TX
  );

  delay(500);
  Serial.println();
  Serial.println("ESP32-CAM SCREENSHOT NODE");

  cameraMutex = xSemaphoreCreateMutex();

  applyFlash(false);

  if (!initCamera()) {
    Serial.println("CAMERA INITIALIZATION FAILED");
  }

  if (connectToMainAP()) {
    startServer();
    delay(100);
    cameraEnabled = false;
    sendState();
  }
  else {
    Serial.println("MAIN AP NOT AVAILABLE");
  }
}

void loop() {
  processUART();

  if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = millis();
    if (WiFi.status() == WL_CONNECTED) {
      sendState();
    }
  }

  if (millis() - lastWiFiCheck >= WIFI_CHECK_MS) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      ensureWiFi();
    }
  }

  delay(1);
}
