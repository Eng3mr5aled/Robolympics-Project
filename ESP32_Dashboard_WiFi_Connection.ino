/*
  ============================================================
  ESP32 DASHBOARD - WIFI CONNECTION ONLY
  ============================================================

  This is the Wi-Fi/AP part of the original robot code.

  The ESP32 creates its own Wi-Fi network. Your phone/laptop
  connects to it, then the dashboard is opened at:

      http://192.168.4.1

  WIFI SETTINGS FROM THE ORIGINAL CODE:
    SSID       : ESP32_Turbo
    Password   : 123456789
    Channel    : 6
    ESP32 IP   : 192.168.4.1
    Gateway    : 192.168.4.1
    Subnet     : 255.255.255.0
    HTTP Port  : 80
    WebSocket  : 81 (used by the full robot dashboard)

  This file intentionally contains NO:
    - motor code
    - gripper code
    - camera code
    - Edge Impulse
    - autonomous mode
    - ultrasonic sensors

  It is only a clean Wi-Fi + dashboard connection test.
  ============================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

// ============================================================
// WIFI SETTINGS
// These values are the same as the original robot code.
// ============================================================

// Name of the Wi-Fi network created by the ESP32.
const char* AP_SSID = "ESP32_Turbo";

// Password required by the phone/laptop to connect.
const char* AP_PASSWORD = "123456789";

// 2.4 GHz Wi-Fi channel used by the robot.
// Valid non-overlapping choices are normally 1, 6, or 11.
#define WIFI_CHANNEL 6

// ============================================================
// ESP32 ACCESS POINT NETWORK
// ============================================================

// Fixed IP address of the ESP32.
// This is the address used to open the dashboard.
IPAddress AP_IP(192, 168, 4, 1);

// The ESP32 is also the gateway because it is the Access Point.
IPAddress AP_GW(192, 168, 4, 1);

// Local subnet mask.
IPAddress AP_MASK(255, 255, 255, 0);

// HTTP server used to serve the dashboard page.
WebServer server(80);

// ============================================================
// LOW-LATENCY WIFI
// Taken from the original robot code.
// This disables Wi-Fi power saving to reduce communication lag.
// ============================================================
void configureLowLatencyWiFi()
{
  // Disable Arduino Wi-Fi sleep.
  WiFi.setSleep(false);

  // Disable ESP32 Wi-Fi power-save mode.
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Same transmit-power setting as the original code.
  esp_wifi_set_max_tx_power(78);
}

// ============================================================
// SIMPLE DASHBOARD TEST PAGE
// This is only a Wi-Fi connection test, not the full robot UI.
// ============================================================
const char DASHBOARD_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Dashboard Wi-Fi Test</title>
  <style>
    body {
      background:#111;
      color:#fff;
      font-family:Arial,sans-serif;
      text-align:center;
      padding:30px;
    }
    .card {
      max-width:450px;
      margin:auto;
      padding:25px;
      background:#222;
      border-radius:15px;
    }
    .ok {
      color:#66e08a;
      font-size:25px;
      font-weight:bold;
    }
    .info {
      color:#aaa;
      line-height:1.8;
    }
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 4WD</h1>
    <div class="ok">Wi-Fi Connected</div>
    <p class="info">
      SSID: ESP32_Turbo<br>
      ESP32 IP: 192.168.4.1<br>
      Wi-Fi Channel: 6<br>
      HTTP Port: 80<br>
      WebSocket Port: 81
    </p>
  </div>
</body>
</html>
)rawliteral";

// ============================================================
// SETUP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 DASHBOARD WIFI TEST");
  Serial.println("====================================");

  // ----------------------------------------------------------
  // 1) Make the ESP32 an Access Point (AP).
  // ----------------------------------------------------------
  //
  // The ESP32 creates its own network:
  //
  //     SSID     = ESP32_Turbo
  //     Password = 123456789
  //     Channel  = 6
  //
  WiFi.mode(WIFI_AP);

  // ----------------------------------------------------------
  // 2) Give the ESP32 the fixed dashboard IP.
  // ----------------------------------------------------------
  //
  // After connecting to ESP32_Turbo, the browser should open:
  //
  //     http://192.168.4.1
  //
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);

  // ----------------------------------------------------------
  // 3) Start the Wi-Fi Access Point.
  // ----------------------------------------------------------
  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    WIFI_CHANNEL
  );

  // ----------------------------------------------------------
  // 4) Apply the same low-latency Wi-Fi settings.
  // ----------------------------------------------------------
  configureLowLatencyWiFi();

  // ----------------------------------------------------------
  // 5) Serve the test dashboard.
  // ----------------------------------------------------------
  server.on("/", HTTP_GET, []()
  {
    server.send(200, "text/html", DASHBOARD_PAGE);
  });

  // Start HTTP server on port 80.
  server.begin();

  // ----------------------------------------------------------
  // 6) Print connection information to Serial Monitor.
  // ----------------------------------------------------------
  Serial.println();
  Serial.println("Wi-Fi AP started successfully.");
  Serial.println("------------------------------------");

  Serial.print("SSID: ");
  Serial.println(AP_SSID);

  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);

  Serial.print("Channel: ");
  Serial.println(WIFI_CHANNEL);

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.softAPIP());

  Serial.println("------------------------------------");
  Serial.println("Connect your phone/laptop to ESP32_Turbo");
  Serial.println("Then open http://192.168.4.1");
  Serial.println();
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
  // Process browser requests for the dashboard.
  server.handleClient();

  // Print the number of connected Wi-Fi clients every 2 sec.
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 2000)
  {
    lastPrint = millis();

    Serial.print("Wi-Fi clients: ");
    Serial.println(WiFi.softAPgetStationNum());
  }
}
