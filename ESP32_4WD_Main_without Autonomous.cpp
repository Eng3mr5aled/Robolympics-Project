// ============================================================ 
// ESP32 4WD ROBOT + ESP32-CAM + GRIPPER 
// LOW-LATENCY / RELIABLE VERSION 
// 
// MAIN ESP32 
//   - Wi-Fi AP + dashboard 
//   - WebSocket real-time motor control 
//   - 4 motor H-bridge PWM 
//   - Gripper servo GPIO 21 using built-in LEDC 
//   - UART link to ESP32-CAM 
//   - Camera IP learned from UART 
//   - Live camera stream shown in dashboard 
//   - Auto recovery after camera/Wi-Fi reconnect 
// 
// KEYBOARD 
//   G = Gripper OPEN 
//   R = Gripper CLOSE 
//   C = Camera ON 
//   D = Camera OFF 
//   V = Reverse controls ON/OFF 
//   Arrows = movement 
//   SPACE = stop 
//   +/- = speed 
// 
// REQUIRED LIBRARY 
//   arduinoWebSockets by Markus Sattler / Links2004 
// 
// CAMERA UART CONNECTION 
//   MAIN GPIO32 TX  ->  ESP32-CAM GPIO3 RX
//   MAIN GPIO33 RX  <-  ESP32-CAM GPIO1 TX
//   MAIN GND        ->  ESP32-CAM GND
// 
// IMPORTANT 
//   Main ESP32 AP and ESP32-CAM STA use the same Wi-Fi network. 
//   Camera uses DHCP and reports its current IP automatically over UART. 
// ============================================================ 
 
#include <Arduino.h> 
#include <WiFi.h> 
#include <WebServer.h> 
#include <WebSocketsServer.h> 
#include <esp_wifi.h> 
 
// ============================================================ 
// WIFI 
// ============================================================ 
 
const char* AP_SSID     = "ESP32_Turbo"; 
const char* AP_PASSWORD = "123456789"; 
 
// Use 1, 6, or 11. Change this if your local 2.4 GHz band is busy. 
#define WIFI_CHANNEL 6 
 
WebServer server(80); 
WebSocketsServer webSocket(81); 
 
IPAddress AP_IP(192, 168, 4, 1); 
IPAddress AP_GW(192, 168, 4, 1); 
IPAddress AP_MASK(255, 255, 255, 0); 
 
// ============================================================ 
// CAMERA UART 
// ============================================================ 
 
HardwareSerial CameraSerial(2); 
 
#define CAMERA_UART_TX   32 
#define CAMERA_UART_RX   33 
#define CAMERA_UART_BAUD 115200 
 
// ============================================================ 
// MOTOR PINS 
// ============================================================ 
 
const int LF_motorF = 22; 
const int LF_motorB = 23; 
 
const int LR_motorF = 25; 
const int LR_motorB = 26; 
 
const int RF_motorF = 4; 
const int RF_motorB = 16; 
 
const int RR_motorF = 17; 
const int RR_motorB = 18; 
 
// ============================================================ 
// GRIPPER SERVO 
// ============================================================ 
 
const int GRIPPER_SERVO_PIN = 21; 
const int GRIPPER_OPEN_ANGLE  = 5; 
const int GRIPPER_CLOSE_ANGLE = 180; 
 
bool gripperClosed = false; 
bool servoReady = false; 
 
// ============================================================ 
// CAMERA UART LINK 
// ============================================================ 
// Main ESP32 remains the Wi-Fi AP at 192.168.4.1. 
// ESP32-CAM uses DHCP and reports its current IP to Main over UART. 
// 
// UART protocol: 
//   Main -> Camera:
//     CAM_ON
//     CAM_OFF
//
//   Camera -> Main:
//     CAM_STATE:ON
//     CAM_STATE:OFF
//     CAM_IP:192.168.4.X
//
// Wi-Fi is still used for:
//   GET /stream
//   GET /capture
//   GET /status
// ============================================================ 
 
// ============================================================ 
// CAMERA STATE 
// ============================================================ 
 
bool cameraActualOn  = false; 
bool cameraDesiredOn = false; 
 
char cameraStatus[16] = "OFF"; 
char detectionText[64] = "OFF"; 
float detectionConfidence = 0.0f; 
 
// Camera IP is learned automatically from the camera UART heartbeat. 
char cameraIP[16] = ""; 
 
volatile unsigned long lastCameraHeartbeat = 0; 
bool cameraLinkKnown = false; 
 
const unsigned long CAMERA_LINK_TIMEOUT = 4000; 
 
// Application-level ON/OFF reliability over UART. 
bool cameraCommandPending = false; 
uint8_t cameraCommandTries = 0; 
unsigned long cameraRetryAt = 0; 
 
const unsigned long CAMERA_RETRY_MS = 400; 
const uint8_t CAMERA_MAX_TRIES = 10; 
 
// Forward declarations for helpers defined later. 
void copyText(char* dst, size_t dstSize, const char* src); 
bool cameraLinkOnline(); 
 
// ============================================================ 
// CAMERA UART COMMAND 
// ============================================================ 
 
void sendCameraUART(const char* command) 
{ 
    if (command == nullptr || command[0] == '\0') { 
        return; 
    } 
 
    CameraSerial.println(command); 
} 
 
void requestCamera(bool turnOn) 
{ 
    cameraDesiredOn = turnOn; 
    cameraCommandPending = true; 
    cameraCommandTries = 1; 
    cameraRetryAt = millis(); 
 
    if (turnOn) { 
        copyText(cameraStatus, sizeof(cameraStatus), "STARTING"); 
        copyText(detectionText, sizeof(detectionText), "WAITING"); 
        detectionConfidence = 0.0f; 
    } 
    else { 
        copyText(cameraStatus, sizeof(cameraStatus), "STOPPING"); 
    } 
 
    // Send immediately. If the camera does not answer, 
    // the retry service will continue in the background. 
    sendCameraUART( 
        turnOn ? "CAM_ON" : "CAM_OFF" 
    ); 
 
    cameraRetryAt = 
        millis() + CAMERA_RETRY_MS; 
} 
 
void serviceCameraRetry() 
{ 
    if (!cameraCommandPending) return; 
    if (millis() < cameraRetryAt) return; 
 
    if (cameraCommandTries >= CAMERA_MAX_TRIES) { 
        cameraCommandPending = false; 
        cameraActualOn = false; 
 
        copyText(cameraStatus, sizeof(cameraStatus), "NO LINK"); 
 
        if (cameraDesiredOn) { 
            copyText(detectionText, sizeof(detectionText), "NO LINK"); 
        } 
        else { 
            copyText(detectionText, sizeof(detectionText), "OFF"); 
        } 
 
        detectionConfidence = 0.0f; 
        return; 
    } 
 
    sendCameraUART( 
        cameraDesiredOn ? "CAM_ON" : "CAM_OFF" 
    ); 
 
    cameraCommandTries++; 
    cameraRetryAt = millis() + CAMERA_RETRY_MS; 
} 
 
void serviceCameraAutoRecovery() 
{ 
    if (!cameraLinkOnline()) return; 
    if (cameraCommandPending) return; 
 
    // After a camera reboot/reconnect, restore the desired state. 
    if (cameraDesiredOn && !cameraActualOn) { 
        requestCamera(true); 
    } 
    else if (!cameraDesiredOn && cameraActualOn) { 
        requestCamera(false); 
    } 
} 
 
// ============================================================ 
// MAIN RECEIVES CAMERA UART 
// ============================================================ 
 
void processCameraUART() 
{ 
    static char buffer[128]; 
    static size_t index = 0; 
 
    while (CameraSerial.available()) { 
 
        char c = (char)CameraSerial.read(); 
 
        if (c == '\n' || c == '\r') { 
 
            if (index == 0) { 
                continue; 
            } 
 
            buffer[index] = '\0'; 
 
            // ------------------------------------------------ 
            // CAMERA ON 
            // ------------------------------------------------ 
 
            if (strcmp(buffer, "CAM_STATE:ON") == 0) { 
 
                cameraLinkKnown = true; 
                lastCameraHeartbeat = millis(); 
                cameraActualOn = true; 
 
                if (cameraDesiredOn) { 
                    cameraCommandPending = false; 
                } 
 
                copyText( 
                    cameraStatus, 
                    sizeof(cameraStatus), 
                    "ON" 
                ); 
 
                if ( 
                    strcmp(detectionText, "OFF") == 0 || 
                    strcmp(detectionText, "NO LINK") == 0 || 
                    detectionText[0] == '\0' 
                ) { 
 
                    copyText( 
                        detectionText, 
                        sizeof(detectionText), 
                        "WAITING" 
                    ); 
 
                    detectionConfidence = 0.0f; 
                } 
            } 
 
            // ------------------------------------------------ 
            // CAMERA OFF 
            // ------------------------------------------------ 
 
            else if (strcmp(buffer, "CAM_STATE:OFF") == 0) { 
 
                cameraLinkKnown = true; 
                lastCameraHeartbeat = millis(); 
                cameraActualOn = false; 
 
                if (!cameraDesiredOn) { 
 
                    cameraCommandPending = false; 
 
                    copyText( 
                        cameraStatus, 
                        sizeof(cameraStatus), 
                        "OFF" 
                    ); 
 
                    copyText( 
                        detectionText, 
                        sizeof(detectionText), 
                        "OFF" 
                    ); 
 
                    detectionConfidence = 0.0f; 
                } 
            } 
 
            // ------------------------------------------------ 
            // CAMERA IP 
            // ------------------------------------------------ 
 
            else if (strncmp(buffer, "CAM_IP:", 7) == 0) { 
 
                const char* ip = buffer + 7; 
 
                if ( 
                    strlen(ip) > 0 && 
                    strlen(ip) < sizeof(cameraIP) 
                ) { 
 
                    copyText( 
                        cameraIP, 
                        sizeof(cameraIP), 
                        ip 
                    ); 
                } 
 
                cameraLinkKnown = true; 
                lastCameraHeartbeat = millis(); 
            } 
 
            index = 0; 
        } 
        else { 
 
            if (index < sizeof(buffer) - 1) { 
                buffer[index++] = c; 
            } 
            else { 
                index = 0; 
            } 
        } 
    } 
} 
 
// ============================================================ 
// MAIN HTTP CAMERA STATUS FOR LAPTOP 
// 
// Laptop uses this to discover the camera's current DHCP IP. 
// ============================================================ 
 
void handleCameraStatus() 
{ 
    char json[192]; 
 
    snprintf( 
        json, 
        sizeof(json), 
        "{\"camera\":\"%s\"," 
        "\"link\":%s," 
        "\"ip\":\"%s\"}", 
        cameraStatus, 
        cameraLinkOnline() ? "true" : "false", 
        cameraIP 
    ); 
 
    server.send( 
        200, 
        "application/json", 
        json 
    ); 
} 
 
// ============================================================ 
// MAIN HTTP RECEIVES EDGE IMPULSE RESULT FROM LAPTOP 
// 
// Expected: 
// /detection?label=Pyramid&confidence=0.94&p=0.94&c=0.03&b=0.02 
// Scores may be sent either as 0..1 or 0..100. 
// ============================================================ 
 
float readScoreArg(const char* name) 
{ 
    if (!server.hasArg(name)) return 0.0f; 
 
    float value = server.arg(name).toFloat(); 
 
    if (value > 1.0f) { 
        value /= 100.0f; 
    } 
 
    return constrain(value, 0.0f, 1.0f); 
} 
 
void handleDetection() 
{ 
    if (!server.hasArg("label")) { 
        server.send( 
            400, 
            "text/plain", 
            "Missing label" 
        ); 
        return; 
    } 
 
    String label = server.arg("label"); 
 
    if (label.length() >= sizeof(detectionText)) { 
        label = label.substring(0, sizeof(detectionText) - 1); 
    } 
 
    float confidence = 0.0f; 
 
    if (server.hasArg("confidence")) { 
        confidence = server.arg("confidence").toFloat(); 
 
        if (confidence > 1.0f) { 
            confidence /= 100.0f; 
        } 
 
        confidence = 
            constrain(confidence, 0.0f, 1.0f); 
    } 
 
    const float pyramidScore = readScoreArg("p"); 
    const float cubeScore    = readScoreArg("c"); 
    const float ballScore    = readScoreArg("b"); 
 
    // Preserve the existing dashboard's P:/C:/B: parser, 
    // while putting the detected shape first. 
    snprintf( 
        detectionText, 
        sizeof(detectionText), 
        "%s | P:%.0f C:%.0f B:%.0f", 
        label.c_str(), 
        pyramidScore * 100.0f, 
        cubeScore * 100.0f, 
        ballScore * 100.0f 
    ); 
 
    detectionConfidence = confidence; 
 
    server.send( 
        200, 
        "text/plain", 
        "OK" 
    ); 
} 
 
// ============================================================ 
// ROBOT CONTROL 
// ============================================================ 
 
int SPEED = 255; 
const int SPEED_STEP = 10; 
 
// Inside side is slowed down instead of reversing. 
const float STEER_RATIO = 0.35f; 
 
char currentCommand = 'S'; 
char robotStatus[20] = "STOPPED"; 
 
unsigned long lastControlMillis = 0; 
const unsigned long CONTROL_TIMEOUT = 450; 
 
// ============================================================ 
// DASHBOARD STATUS 
// ============================================================ 
 
unsigned long lastStatusBroadcast = 0; 
const unsigned long STATUS_INTERVAL = 50; 
 
// ============================================================ 
// HELPER 
// ============================================================ 
 
void copyText(char* dst, size_t dstSize, const char* src) 
{ 
    if (dstSize == 0) return; 
    snprintf(dst, dstSize, "%s", src ? src : ""); 
} 
 
bool cameraLinkOnline() 
{ 
    return cameraLinkKnown && 
           (millis() - lastCameraHeartbeat <= CAMERA_LINK_TIMEOUT); 
} 
 
void configureLowLatencyWiFi() 
{ 
    WiFi.setSleep(false); 
    esp_wifi_set_ps(WIFI_PS_NONE); 
    esp_wifi_set_max_tx_power(78); 
} 
 
// ============================================================ 
// SERVO 
// ============================================================ 
 
void servoWriteAngle(int angle) 
{ 
    if (!servoReady) return; 
 
    angle = constrain(angle, 0, 180); 
 
    const int pulseWidthUs = map(angle, 0, 180, 500, 2400); 
 
    const uint32_t duty = 
        ((uint32_t)pulseWidthUs * 65535UL) / 20000UL; 
 
    ledcWrite(GRIPPER_SERVO_PIN, duty); 
} 
 
void gripperOpen() 
{ 
    servoWriteAngle(GRIPPER_OPEN_ANGLE); 
    gripperClosed = false; 
} 
 
void gripperClose() 
{ 
    servoWriteAngle(GRIPPER_CLOSE_ANGLE); 
    gripperClosed = true; 
} 
 
// ============================================================ 
// MOTOR CONTROL 
// ============================================================ 
 
void setMotor(int forwardPin, int reversePin, int forwardPwm, int reversePwm) 
{ 
    analogWrite(forwardPin, constrain(forwardPwm, 0, 255)); 
    analogWrite(reversePin, constrain(reversePwm, 0, 255)); 
} 
 
void stopAll() 
{ 
    setMotor(LF_motorF, LF_motorB, 0, 0); 
    setMotor(LR_motorF, LR_motorB, 0, 0); 
    setMotor(RF_motorF, RF_motorB, 0, 0); 
    setMotor(RR_motorF, RR_motorB, 0, 0); 
 
    currentCommand = 'S'; 
    copyText(robotStatus, sizeof(robotStatus), "STOPPED"); 
} 
 
void moveForward() 
{ 
    setMotor(LF_motorF, LF_motorB, SPEED, 0); 
    setMotor(LR_motorF, LR_motorB, SPEED, 0); 
    setMotor(RF_motorF, RF_motorB, SPEED, 0); 
    setMotor(RR_motorF, RR_motorB, SPEED, 0); 
 
    copyText(robotStatus, sizeof(robotStatus), "FORWARD"); 
} 
 
void moveBackward() 
{ 
    setMotor(LF_motorF, LF_motorB, 0, SPEED); 
    setMotor(LR_motorF, LR_motorB, 0, SPEED); 
    setMotor(RF_motorF, RF_motorB, 0, SPEED); 
    setMotor(RR_motorF, RR_motorB, 0, SPEED); 
 
    copyText(robotStatus, sizeof(robotStatus), "BACKWARD"); 
} 
 
void turnLeft() 
{ 
    setMotor(LF_motorF, LF_motorB, 0, SPEED); 
    setMotor(LR_motorF, LR_motorB, 0, SPEED); 
    setMotor(RF_motorF, RF_motorB, SPEED, 0); 
    setMotor(RR_motorF, RR_motorB, SPEED, 0); 
 
    copyText(robotStatus, sizeof(robotStatus), "LEFT"); 
} 
 
void turnRight() 
{ 
    setMotor(LF_motorF, LF_motorB, SPEED, 0); 
    setMotor(LR_motorF, LR_motorB, SPEED, 0); 
    setMotor(RF_motorF, RF_motorB, 0, SPEED); 
    setMotor(RR_motorF, RR_motorB, 0, SPEED); 
 
    copyText(robotStatus, sizeof(robotStatus), "RIGHT"); 
} 
 
// ============================================================ 
// COMBINED STEERING 
// ============================================================ 
 
void executeCommand(char command); 
 
void moveForwardLeft() 
{ 
    const int insideSpeed = (int)(SPEED * STEER_RATIO); 
 
    setMotor(LF_motorF, LF_motorB, insideSpeed, 0); 
    setMotor(LR_motorF, LR_motorB, insideSpeed, 0); 
    setMotor(RF_motorF, RF_motorB, SPEED, 0); 
    setMotor(RR_motorF, RR_motorB, SPEED, 0); 
 
    currentCommand = 'F'; 
    copyText(robotStatus, sizeof(robotStatus), "FORWARD LEFT"); 
} 
 
void moveForwardRight() 
{ 
    const int insideSpeed = (int)(SPEED * STEER_RATIO); 
 
    setMotor(LF_motorF, LF_motorB, SPEED, 0); 
    setMotor(LR_motorF, LR_motorB, SPEED, 0); 
    setMotor(RF_motorF, RF_motorB, insideSpeed, 0); 
    setMotor(RR_motorF, RR_motorB, insideSpeed, 0); 
 
    currentCommand = 'F'; 
    copyText(robotStatus, sizeof(robotStatus), "FORWARD RIGHT"); 
} 
 
void moveBackwardLeft() 
{ 
    const int insideSpeed = (int)(SPEED * STEER_RATIO); 
 
    setMotor(LF_motorF, LF_motorB, 0, insideSpeed); 
    setMotor(LR_motorF, LR_motorB, 0, insideSpeed); 
    setMotor(RF_motorF, RF_motorB, 0, SPEED); 
    setMotor(RR_motorF, RR_motorB, 0, SPEED); 
 
    currentCommand = 'B'; 
    copyText(robotStatus, sizeof(robotStatus), "BACKWARD LEFT"); 
} 
 
void moveBackwardRight() 
{ 
    const int insideSpeed = (int)(SPEED * STEER_RATIO); 
 
    setMotor(LF_motorF, LF_motorB, 0, SPEED); 
    setMotor(LR_motorF, LR_motorB, 0, SPEED); 
    setMotor(RF_motorF, RF_motorB, 0, insideSpeed); 
    setMotor(RR_motorF, RR_motorB, 0, insideSpeed); 
 
    currentCommand = 'B'; 
    copyText(robotStatus, sizeof(robotStatus), "BACKWARD RIGHT"); 
} 
 
void executeMovementCommand(const char* command) 
{ 
    if (command == nullptr || command[0] == '\0') return; 
 
    if (strcmp(command, "FL") == 0) { 
        moveForwardLeft(); 
    } 
    else if (strcmp(command, "FR") == 0) { 
        moveForwardRight(); 
    } 
    else if (strcmp(command, "BL") == 0) { 
        moveBackwardLeft(); 
    } 
    else if (strcmp(command, "BR") == 0) { 
        moveBackwardRight(); 
    } 
    else if (strlen(command) == 1) { 
        executeCommand(command[0]); 
        return; 
    } 
    else { 
        return; 
    } 
 
    lastControlMillis = millis(); 
} 
 
void executeCommand(char command) 
{ 
    switch (command) { 
        case 'F': 
            moveForward(); 
            currentCommand = 'F'; 
            break; 
 
        case 'B': 
            moveBackward(); 
            currentCommand = 'B'; 
            break; 
 
        case 'L': 
            turnLeft(); 
            currentCommand = 'L'; 
            break; 
 
        case 'R': 
            turnRight(); 
            currentCommand = 'R'; 
            break; 
 
        case 'S': 
        default: 
            stopAll(); 
            break; 
    } 
 
    lastControlMillis = millis(); 
} 
 
// ============================================================ 
// Re-apply the currently active movement using the new SPEED value.
void reapplyCurrentMovement()
{
    if (currentCommand == 'S') return;

    if (strcmp(robotStatus, "FORWARD LEFT") == 0) moveForwardLeft();
    else if (strcmp(robotStatus, "FORWARD RIGHT") == 0) moveForwardRight();
    else if (strcmp(robotStatus, "BACKWARD LEFT") == 0) moveBackwardLeft();
    else if (strcmp(robotStatus, "BACKWARD RIGHT") == 0) moveBackwardRight();
    else {
        switch (currentCommand) {
            case 'F': moveForward(); break;
            case 'B': moveBackward(); break;
            case 'L': turnLeft(); break;
            case 'R': turnRight(); break;
            default: stopAll(); return;
        }
    }
    lastControlMillis = millis();
}

// DASHBOARD STATUS JSON 
// ============================================================ 
 
void makeStatusJson(char* out, size_t outSize) 
{ 
    const bool camLink = cameraLinkOnline(); 
 
    snprintf( 
        out, 
        outSize, 
        "{\"robot\":\"%s\"," 
        "\"camera\":\"%s\"," 
        "\"detection\":\"%s\"," 
        "\"confidence\":%.2f," 
        "\"gripper\":\"%s\"," 
        "\"clients\":%u," 
        "\"channel\":%d," 
        "\"camLink\":%s," 
        "\"cameraIP\":\"%s\"," 
        "\"speed\":%d}", 
        robotStatus, 
        cameraStatus, 
        detectionText, 
        detectionConfidence, 
        gripperClosed ? "CLOSED" : "OPEN", 
        WiFi.softAPgetStationNum(), 
        WIFI_CHANNEL, 
        camLink ? "true" : "false", 
        cameraIP, 
        SPEED 
    ); 
} 
 
void sendStatusToClient(uint8_t clientNum) 
{ 
    char json[448]; 
    makeStatusJson(json, sizeof(json)); 
    webSocket.sendTXT(clientNum, json); 
} 
 
void broadcastStatus() 
{ 
    if (webSocket.connectedClients() == 0) return; 
 
    char json[448]; 
    makeStatusJson(json, sizeof(json)); 
    webSocket.broadcastTXT(json); 
} 
 
// ============================================================ 
// WEBSOCKET COMMAND PARSER 
// ============================================================ 
 
void handleWebSocketMessage(uint8_t clientNum, const char* msg) 
{ 
    if (msg == nullptr || msg[0] == '\0') return; 
 
    if (strncmp(msg, "PING:", 5) == 0) { 
        char reply[64]; 
        snprintf(reply, sizeof(reply), "PONG:%s", msg + 5); 
        webSocket.sendTXT(clientNum, reply); 
        return; 
    } 
 
    if (strcmp(msg, "HELLO") == 0) { 
        sendStatusToClient(clientNum); 
        return; 
    } 
 
    // Movement commands are reserved for dashboard arrow handling. 
    if (strcmp(msg, "F") == 0 || 
        strcmp(msg, "B") == 0 || 
        strcmp(msg, "L") == 0 || 
        strcmp(msg, "R") == 0 || 
        strcmp(msg, "S") == 0 || 
        strcmp(msg, "FL") == 0 || 
        strcmp(msg, "FR") == 0 || 
        strcmp(msg, "BL") == 0 || 
        strcmp(msg, "BR") == 0) { 
 
        executeMovementCommand(msg); 
        return; 
    } 
 
    if (strncmp(msg, "SPD:", 4) == 0) {
        int newSpeed = atoi(msg + 4);
        SPEED = constrain(newSpeed, 0, 255);
        reapplyCurrentMovement();
        broadcastStatus();
        return;
    } 
 
    if (strcmp(msg, "CAM:ON") == 0) { 
        requestCamera(true); 
        return; 
    } 
 
    if (strcmp(msg, "CAM:OFF") == 0) { 
        requestCamera(false); 
        return; 
    } 
 
    if (strcmp(msg, "GRIP:OPEN") == 0) { 
        gripperOpen(); 
        return; 
    } 
 
    if (strcmp(msg, "GRIP:CLOSE") == 0) { 
        gripperClose(); 
        return; 
    } 
} 
 
void webSocketEvent( 
    uint8_t clientNum, 
    WStype_t type, 
    uint8_t* payload, 
    size_t length 
) 
{ 
    switch (type) { 
        case WStype_CONNECTED: 
            sendStatusToClient(clientNum); 
            break; 
 
        case WStype_DISCONNECTED: 
            // Firmware timeout remains the final movement safety layer. 
            break; 
 
        case WStype_TEXT: { 
            char msg[80]; 
            size_t copyLen = min(length, sizeof(msg) - 1); 
            memcpy(msg, payload, copyLen); 
            msg[copyLen] = '\0'; 
            handleWebSocketMessage(clientNum, msg); 
            break; 
        } 
 
        default: 
            break; 
    } 
} 
 
// ============================================================ 
// DASHBOARD HTML 
// ============================================================ 
 
const char MAIN_page[] PROGMEM = R"rawliteral( 
<!DOCTYPE html> 
<html> 
<head> 
<meta charset="utf-8"> 
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no"> 
<meta name="theme-color" content="#111111"> 
<title>ESP32 4WD Robot</title> 
<style> 
*{box-sizing:border-box;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none} 
body{margin:0;background:#111;color:#fff;font-family:Arial,sans-serif;text-align:center;overscroll-behavior:none} 
h1{margin:16px 0 8px;font-size:28px} 
.card{width:92%;max-width:430px;margin:12px auto;padding:15px;background:#222;border-radius:14px} 
.main-status{font-size:24px;font-weight:bold;margin-top:4px} 
.small{font-size:13px;color:#aaa;margin-top:7px;line-height:1.6} 
.pill{display:inline-block;padding:4px 8px;margin:2px;border-radius:99px;background:#333} 
.good{color:#66e08a}.warn{color:#ffd166}.bad{color:#ff6b6b} 
.speed-value{font-size:23px;font-weight:bold;margin:8px} 
#speed{width:100%} 
.speed-buttons{display:flex;justify-content:center;gap:12px;margin-top:10px} 
.speed-buttons button{width:92px;height:48px} 
.controls{width:310px;max-width:92vw;margin:20px auto;display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:82px 82px 82px;gap:9px} 
button{border:0;border-radius:16px;background:#333;color:#fff;font-size:22px;font-weight:bold;touch-action:none} 
button:active{transform:scale(.97);background:#555} 
.forward{grid-column:2;grid-row:1}.left{grid-column:1;grid-row:2}.stop{grid-column:2;grid-row:2;background:#b00020;font-size:17px}.right{grid-column:3;grid-row:2}.backward{grid-column:2;grid-row:3} 
.row{display:flex;justify-content:center;gap:10px;flex-wrap:wrap} 
.reverse-control{width:100%;height:52px;background:#444;font-size:16px} 
.reverse-control.active{background:#7b2cbf} 
.row button{width:145px;height:50px;font-size:15px} 
.green{background:#087f23}.red{background:#b00020} 
.detection{font-size:28px;font-weight:bold;margin-top:7px}.confidence{font-size:17px;color:#aaa;margin-top:5px} 
.class-probabilities{ 
  margin-top:12px; 
  padding:10px; 
  background:#181818; 
  border-radius:10px; 
  font-size:17px; 
  line-height:1.8; 
  text-align:left; 
} 
 
.class-probabilities span{ 
  float:right; 
  font-weight:bold; 
} 
 
.keys{line-height:1.8;color:#bbb;font-size:14px} 
.key{display:inline-block;padding:3px 8px;margin:2px;border-radius:6px;background:#333;color:#fff;font-weight:bold} 
.camera-box{position:relative;width:100%;aspect-ratio:4/3;margin-top:12px;background:#000;border-radius:12px;overflow:hidden} 
#cameraStream{width:100%;height:100%;display:block;object-fit:contain;background:#000} 
#cameraOverlay{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;background:#000;color:#aaa;font-size:18px;font-weight:bold} 
</style> 
</head> 
<body> 
<h1>ESP32 4WD</h1> 
 
<div class="card"> 
  <div>Robot Status</div> 
  <div id="status" class="main-status">STOPPED</div> 
  <div class="small"> 
   
    <span class="pill">WS: <span id="wsState">CONNECTING</span></span> 
    <span class="pill">RTT: <span id="rtt">--</span></span> 
    <span class="pill">Wi-Fi: <span id="wifiInfo">--</span></span> 
  </div> 
</div> 
 
<div class="card"> 
  <div>Speed</div> 
  <input id="speed" type="range" min="0" max="255" value="255"> 
  <div id="speedValue" class="speed-value">255</div> 
  <div class="speed-buttons"> 
    <button id="speedDown">-</button> 
    <button id="speedUp">+</button> 
  </div> 
</div> 
 
<div class="controls"> 
  <button class="forward" id="forward">&#9650;</button> 
  <button class="left" id="left">&#9664;</button> 
  <button class="stop" id="stop">STOP</button> 
  <button class="right" id="right">&#9654;</button> 
  <button class="backward" id="backward">&#9660;</button> 
</div> 
 
<div class="card"> 
  <div>Control Direction</div> 
  <div id="reverseStatus" class="main-status">NORMAL</div> 
  <div class="small">V or the button swaps Forward/Backward only. Left/Right stay the same.</div> 
  <div class="row" style="margin-top:12px"> 
    <button id="reverseControls" class="reverse-control">REVERSE CONTROLS: OFF</button> 
  </div> 
</div> 
 
<div class="card"> 
  <div>Camera</div> 
  <div id="cameraStatus" class="main-status">OFF</div> 
  <div class="small"> 
    Camera Link: <span id="camLink">OFFLINE</span><br> 
    Camera IP: <span id="cameraIP">--</span> 
  </div> 
 
  <div class="camera-box"> 
    <img id="cameraStream" alt="Camera Stream" draggable="false"> 
    <div id="cameraOverlay">CAMERA OFF</div> 
  </div> 
 
  <div class="row" style="margin-top:12px"> 
    <button id="cameraOn" class="green">CAMERA ON</button> 
    <button id="cameraOff" class="red">CAMERA OFF</button> 
  </div> 
</div> 
 
<div class="card"> 
  <div>Edge Impulse Detection</div> 
  <div id="detection" class="detection">OFF</div> 
  <div id="confidence" class="confidence">--</div> 
 
<div id="classProbabilities" class="class-probabilities"> 
  <div>Pyramid: <span id="pyramidProb">--</span></div> 
  <div>Cube: <span id="cubeProb">--</span></div> 
  <div>Ball: <span id="ballProb">--</span></div> 
</div> 
 
</div> 
 
<div class="card"> 
  <div>Gripper</div> 
  <div id="gripperStatus" class="main-status">OPEN</div> 
  <div class="row" style="margin-top:12px"> 
    <button id="gripperOpen" class="green">GRIPPER OPEN [G]</button> 
    <button id="gripperClose" class="red">GRIPPER CLOSE [R]</button> 
  </div> 
</div> 
 
<div class="card keys"> 
  <b>Keyboard Controls</b><br><br> 
  <span class="key">&#8593;</span> Forward 
  <span class="key">&#8595;</span> Backward<br> 
  <span class="key">&#8592;</span> Left 
  <span class="key">&#8594;</span> Right<br><br> 
  <span class="key">G</span> Gripper Open 
  <span class="key">R</span> Gripper Close<br> 
  <span class="key">C</span> Camera ON 
  <span class="key">D</span> Camera OFF<br> 
  <span class="key">V</span> Reverse ON/OFF<br><br> 
  <span class="key">SPACE</span> Stop<br> 
  <span class="key">+</span> Speed Up 
  <span class="key">-</span> Speed Down<br><br> 
  <small>Hold Up + Left / Up + Right for combined steering</small> 
</div> 
 
<script> 
'use strict'; 
 
let ws = null; 
let reconnectTimer = null; 
const pressedKeys = new Set(); 
const pressedButtons = new Set(); 
let currentMovement = ''; 
let currentSpeed = 255; 
const SPEED_STEP = 10; 
 
let reverseControls = localStorage.getItem('reverseControls_v2') === 'true'; 
let currentCameraStreamURL = ''; 
let cameraReconnectTimer = null; 
 
const $ = id => document.getElementById(id); 
 
const statusEl = $('status'); 
const wsStateEl = $('wsState'); 
const rttEl = $('rtt'); 
const wifiInfoEl = $('wifiInfo'); 
const speedEl = $('speed'); 
const speedValueEl = $('speedValue'); 
const cameraEl = $('cameraStatus'); 
const camLinkEl = $('camLink'); 
const cameraIPEl = $('cameraIP'); 
const cameraStreamEl = $('cameraStream'); 
const cameraOverlayEl = $('cameraOverlay'); 
const detectionEl = $('detection'); 
const confidenceEl = $('confidence'); 
 
const pyramidProbEl = $('pyramidProb'); 
const cubeProbEl = $('cubeProb'); 
const ballProbEl = $('ballProb'); 
const gripperEl = $('gripperStatus'); 
const reverseControlsEl = $('reverseControls'); 
const reverseStatusEl = $('reverseStatus'); 
 
function updateReverseControlsUI(){ 
  if(reverseControls){ 
    reverseStatusEl.textContent = 'REVERSED'; 
    reverseControlsEl.textContent = 'REVERSE CONTROLS: ON [V]'; 
    reverseControlsEl.classList.add('active'); 
  }else{ 
    reverseStatusEl.textContent = 'NORMAL'; 
    reverseControlsEl.textContent = 'REVERSE CONTROLS: OFF [V]'; 
    reverseControlsEl.classList.remove('active'); 
  } 
} 
 
function applyReverseCommand(cmd){ 
  if(!reverseControls) return cmd; 
 
  switch(cmd){ 
    case 'F': return 'B'; 
    case 'B': return 'F'; 
    case 'L': return 'L'; 
    case 'R': return 'R'; 
    case 'FL': return 'BR'; 
    case 'FR': return 'BL'; 
    case 'BL': return 'FR'; 
    case 'BR': return 'FL'; 
    default: return cmd; 
  } 
} 
 
function setWSState(text, cls=''){ 
  wsStateEl.textContent = text; 
  wsStateEl.className = cls; 
} 
 
function send(msg){ 
  if(ws && ws.readyState === WebSocket.OPEN){ 
    ws.send(msg); 
  } 
} 
 
function stopRobot(){ 
  pressedKeys.clear(); 
  pressedButtons.clear(); 
  currentMovement = ''; 
  send('S'); 
} 
 
function getMovementFromInputs(){ 
  const up = pressedKeys.has('ArrowUp') || pressedButtons.has('forward'); 
  const down = pressedKeys.has('ArrowDown') || pressedButtons.has('backward'); 
  const left = pressedKeys.has('ArrowLeft') || pressedButtons.has('left'); 
  const right = pressedKeys.has('ArrowRight') || pressedButtons.has('right'); 
 
  if(up && !down && left && !right) return ['FL','FORWARD LEFT']; 
  if(up && !down && right && !left) return ['FR','FORWARD RIGHT']; 
  if(down && !up && left && !right) return ['BL','BACKWARD LEFT']; 
  if(down && !up && right && !left) return ['BR','BACKWARD RIGHT']; 
 
  if(up && !down) return ['F','FORWARD']; 
  if(down && !up) return ['B','BACKWARD']; 
  if(left && !right) return ['L','LEFT']; 
  if(right && !left) return ['R','RIGHT']; 
 
  return ['','STOPPED']; 
} 
 
function updateMovement(){ 
  const [rawCmd,label] = getMovementFromInputs(); 
  const cmd = applyReverseCommand(rawCmd); 
 
  if(cmd !== currentMovement){ 
    currentMovement = cmd; 
 
    if(reverseControls && rawCmd){ 
      statusEl.textContent = label + ' (REVERSED)'; 
    }else{ 
      statusEl.textContent = label; 
    } 
 
    send(cmd || 'S'); 
  } 
} 
 
function setSpeedLocal(v){
  currentSpeed = Math.max(0, Math.min(255, v|0));
  speedEl.value = currentSpeed;
  speedValueEl.textContent = currentSpeed;
  send('SPD:' + currentSpeed);
} 
 
function clearCameraStream(){ 
  currentCameraStreamURL = ''; 
  cameraStreamEl.removeAttribute('src'); 
  cameraOverlayEl.style.display = 'flex'; 
} 
 
function setCameraStream(ip){ 
  if(!ip) return; 
 
  const url = 'http://' + ip + '/stream'; 
 
  if(currentCameraStreamURL !== url){ 
    currentCameraStreamURL = url; 
    cameraStreamEl.src = url; 
  } 
} 
 
function updateCameraStream(cameraState, camLink, ip){ 
  cameraIPEl.textContent = ip || '--'; 
 
  if(cameraState === 'ON' && camLink && ip){ 
    cameraOverlayEl.style.display = 'none'; 
    setCameraStream(ip); 
  } 
  else{ 
    clearCameraStream(); 
 
    if(cameraState === 'OFF'){ 
      cameraOverlayEl.textContent = 'CAMERA OFF'; 
    } 
    else if(!camLink){ 
      cameraOverlayEl.textContent = 'CAMERA NO LINK'; 
    } 
    else{ 
      cameraOverlayEl.textContent = 'STARTING CAMERA...'; 
    } 
  } 
} 
 
cameraStreamEl.onerror = ()=>{ 
  if(!currentCameraStreamURL) return; 
 
  clearTimeout(cameraReconnectTimer); 
 
  cameraReconnectTimer = setTimeout(()=>{ 
    if(currentCameraStreamURL){ 
      cameraStreamEl.src = currentCameraStreamURL + '?r=' + Date.now(); 
      cameraOverlayEl.style.display = 'none'; 
    } 
  }, 500); 
}; 
 
function updateProbabilities(text){ 
  if(!text){ 
    pyramidProbEl.textContent = '--'; 
    cubeProbEl.textContent = '--'; 
    ballProbEl.textContent = '--'; 
    return; 
  } 
 
  const p = text.match(/P:(\d+(?:\.\d+)?)/); 
  const c = text.match(/C:(\d+(?:\.\d+)?)/); 
  const b = text.match(/B:(\d+(?:\.\d+)?)/); 
 
  pyramidProbEl.textContent = p ? p[1] + '%' : '--'; 
  cubeProbEl.textContent    = c ? c[1] + '%' : '--'; 
  ballProbEl.textContent    = b ? b[1] + '%' : '--'; 
} 
 
function connectWS(){ 
  clearTimeout(reconnectTimer); 
 
  const host = location.hostname || '192.168.4.1'; 
  ws = new WebSocket('ws://' + host + ':81/'); 
 
  ws.onopen = ()=>{ 
    setWSState('CONNECTED','good'); 
    send('HELLO'); 
    updateMovement(); 
  }; 
 
  ws.onclose = ()=>{ 
    setWSState('OFFLINE','bad'); 
    pressedKeys.clear(); 
    pressedButtons.clear(); 
    currentMovement = ''; 
    statusEl.textContent = 'DISCONNECTED'; 
    clearCameraStream(); 
    reconnectTimer = setTimeout(connectWS, 300); 
  }; 
 
  ws.onerror = ()=>{ 
    setWSState('ERROR','bad'); 
  }; 
 
  ws.onmessage = event=>{ 
    const text = String(event.data); 
 
    if(text.startsWith('PONG:')){ 
      const sent = Number(text.substring(5)); 
      if(!Number.isNaN(sent)){ 
        rttEl.textContent = 
          Math.max(0, Math.round(performance.now() - sent)) + ' ms'; 
      } 
      return; 
    } 
 
    try{ 
      const d = JSON.parse(text); 
 
      statusEl.textContent = d.robot; 
      cameraEl.textContent = d.camera; 
      detectionEl.textContent = d.detection; 
      updateProbabilities(d.detection); 
      currentSpeed = d.speed; 
      speedEl.value = d.speed; 
      speedValueEl.textContent = d.speed; 
      gripperEl.textContent = d.gripper; 
      camLinkEl.textContent = d.camLink ? 'ONLINE' : 'OFFLINE'; 
      wifiInfoEl.textContent = 
        d.clients + ' client(s), CH ' + d.channel; 
 
      updateCameraStream( 
        d.camera, 
        d.camLink, 
        d.cameraIP 
      ); 
 
      if(d.confidence > 0){ 
        confidenceEl.textContent = 
          'Confidence: ' + Math.round(d.confidence * 100) + '%'; 
      }else{ 
        confidenceEl.textContent = '--'; 
      } 
    }catch(_){ } 
  }; 
} 
 
// ========================================================
// HOLD MOVEMENT BUTTONS
// ========================================================
 
function bindMove(id){
  const button = $(id);
 
  button.addEventListener('pointerdown', event=>{
    event.preventDefault();
    try{ button.setPointerCapture(event.pointerId); }catch(_){ }
    pressedButtons.add(id);
    updateMovement();
  });
 
  const release = event=>{
    event.preventDefault();
    pressedButtons.delete(id);
    updateMovement();
  };
 
  button.addEventListener('pointerup', release);
  button.addEventListener('pointercancel', release);
  button.addEventListener('lostpointercapture', release);
  button.addEventListener('contextmenu', event=>event.preventDefault());
}
 
bindMove('forward');
bindMove('backward');
bindMove('left');
bindMove('right');
 
$('stop').addEventListener('pointerdown', event=>{
  event.preventDefault();
  stopRobot();
});
 
function toggleReverse(){
  reverseControls = !reverseControls;
  localStorage.setItem(
    'reverseControls_v2',
    String(reverseControls)
  );
 
  send('S');
  pressedKeys.clear();
  pressedButtons.clear();
  currentMovement = '';
 
  updateReverseControlsUI();
  updateMovement();
}
 
reverseControlsEl.addEventListener('pointerdown', event=>{
  event.preventDefault();
  toggleReverse();
});
 
// ========================================================
// KEYBOARD
// ========================================================
 
const movementKeys = new Set([
  'ArrowUp',
  'ArrowDown',
  'ArrowLeft',
  'ArrowRight'
]);
 
document.addEventListener('keydown', event=>{
 
  if(movementKeys.has(event.key)){
    event.preventDefault();
 
    if(!pressedKeys.has(event.key)){
      pressedKeys.add(event.key);
      updateMovement();
    }
 
    return;
  }
 
  if(event.code === 'Space'){
    event.preventDefault();
    if(!event.repeat) stopRobot();
    return;
  }
 
  if(event.repeat) return;
 
  const k = event.key.toUpperCase();
 
  if(k === 'G'){
    event.preventDefault();
    send('GRIP:OPEN');
    return;
  }
 
  if(k === 'R'){
    event.preventDefault();
    send('GRIP:CLOSE');
    return;
  }
 
  if(k === 'C'){
    event.preventDefault();
    cameraEl.textContent = 'STARTING';
    detectionEl.textContent = 'WAITING';
    confidenceEl.textContent = '--';
    send('CAM:ON');
    return;
  }
 
  if(k === 'D'){
    event.preventDefault();
    send('CAM:OFF');
    return;
  }
 
  if(k === 'V'){
    event.preventDefault();
    toggleReverse();
    return;
  }
 
  if(event.key === '+' || event.key === '='){
    event.preventDefault();
    setSpeedLocal(currentSpeed + SPEED_STEP);
    return;
  }
 
  if(event.key === '-' || event.key === '_'){
    event.preventDefault();
    setSpeedLocal(currentSpeed - SPEED_STEP);
    return;
  }
});
 
document.addEventListener('keyup', event=>{
  if(movementKeys.has(event.key)){
    event.preventDefault();
    pressedKeys.delete(event.key);
    updateMovement();
  }
});
 
window.addEventListener('blur', ()=>{
  if(currentMovement) stopRobot();
});
 
document.addEventListener('visibilitychange', ()=>{
  if(document.hidden && currentMovement) stopRobot();
});
 
// Keep movement alive. Firmware has a 450 ms failsafe.
setInterval(()=>{
  if(currentMovement) send(currentMovement);
}, 100);
 
// RTT measurement.
setInterval(()=>{
  if(ws && ws.readyState === WebSocket.OPEN){
    send('PING:' + performance.now());
  }
}, 1000);
 
// ========================================================
// SPEED
// ========================================================
 
speedEl.addEventListener('input', ()=>{
  setSpeedLocal(parseInt(speedEl.value,10));
});
 
$('speedUp').addEventListener('pointerdown', event=>{
  event.preventDefault();
  setSpeedLocal(currentSpeed + SPEED_STEP);
});
 
$('speedDown').addEventListener('pointerdown', event=>{
  event.preventDefault();
  setSpeedLocal(currentSpeed - SPEED_STEP);
});
 
// ========================================================
// CAMERA BUTTONS
// ========================================================
 
$('cameraOn').addEventListener('pointerdown', event=>{
  event.preventDefault();
  cameraEl.textContent = 'STARTING';
  detectionEl.textContent = 'WAITING';
  confidenceEl.textContent = '--';
  send('CAM:ON');
});
 
$('cameraOff').addEventListener('pointerdown', event=>{
  event.preventDefault();
  send('CAM:OFF');
});
 
// ========================================================
// GRIPPER BUTTONS
// ========================================================
 
$('gripperOpen').addEventListener('pointerdown', event=>{
  event.preventDefault();
  send('GRIP:OPEN');
});
 
$('gripperClose').addEventListener('pointerdown', event=>{
  event.preventDefault();
  send('GRIP:CLOSE');
});
 
updateReverseControlsUI();
connectWS();
</script>
</body>
</html>
)rawliteral"; 
 
// ============================================================
// HTTP ROOT
// ============================================================
 
void handleRoot()
{
    server.sendHeader(
        "Cache-Control",
        "no-store, no-cache, must-revalidate, max-age=0"
    );
    server.sendHeader("Pragma", "no-cache");
    server.send_P(200, "text/html", MAIN_page);
}
 
// ============================================================
// SETUP
// ============================================================
 
void setup()
{
    Serial.begin(115200);
    delay(300);
 
    // ========================================================
    // CAMERA UART
    // ========================================================
 
    CameraSerial.begin(
        CAMERA_UART_BAUD,
        SERIAL_8N1,
        CAMERA_UART_RX,
        CAMERA_UART_TX
    );
 
    // Motor GPIO.
    pinMode(LF_motorF, OUTPUT);
    pinMode(LF_motorB, OUTPUT);
    pinMode(LR_motorF, OUTPUT);
    pinMode(LR_motorB, OUTPUT);
    pinMode(RF_motorF, OUTPUT);
    pinMode(RF_motorB, OUTPUT);
    pinMode(RR_motorF, OUTPUT);
    pinMode(RR_motorB, OUTPUT);
 
    stopAll();
 
    // Gripper servo.
    servoReady = ledcAttach(
        GRIPPER_SERVO_PIN,
        50,
        16
    );
 
    if (servoReady) {
        gripperOpen();
    }
 
    // Wi-Fi AP for dashboard, camera, and laptop.
    WiFi.mode(WIFI_AP_STA);
 
    WiFi.softAPConfig(
        AP_IP,
        AP_GW,
        AP_MASK
    );
 
    bool apOK = WiFi.softAP(
        AP_SSID,
        AP_PASSWORD,
        WIFI_CHANNEL,
        false,
        4
    );
 
    // Lock channel after AP starts.
    esp_wifi_set_channel(
        WIFI_CHANNEL,
        WIFI_SECOND_CHAN_NONE
    );
 
    configureLowLatencyWiFi();
 
    server.on(
        "/",
        HTTP_GET,
        handleRoot
    );
 
    // Laptop discovers the camera DHCP IP.
    // Camera IP is learned from UART.
    server.on(
        "/camera/status",
        HTTP_GET,
        handleCameraStatus
    );
 
    // Laptop/Edge Impulse -> Main detection result.
    server.on(
        "/detection",
        HTTP_GET,
        handleDetection
    );
 
    server.onNotFound([](){
        server.send(
            404,
            "text/plain",
            "404"
        );
    });
 
    server.begin();
 
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
 
    lastControlMillis = millis();
 
    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 4WD LOW-LATENCY CONTROLLER");
    Serial.println("========================================");
    Serial.printf("AP: %s\n", AP_SSID);
    Serial.printf("Password: %s\n", AP_PASSWORD);
    Serial.printf(
        "Dashboard: http://%s/\n",
        WiFi.softAPIP().toString().c_str()
    );
    Serial.printf(
        "WebSocket: ws://%s:81/\n",
        WiFi.softAPIP().toString().c_str()
    );
    Serial.printf("Channel: %d\n", WIFI_CHANNEL);
    Serial.printf(
        "AP MAC: %s\n",
        WiFi.softAPmacAddress().c_str()
    );
    Serial.printf(
        "AP started: %s\n",
        apOK ? "YES" : "NO"
    );
    Serial.println("Camera control: UART");
    Serial.println("Camera IP: learned automatically from UART");
    Serial.println("UART TX: GPIO32");
    Serial.println("UART RX: GPIO33");
    Serial.printf(
        "Servo GPIO: %d\n",
        GRIPPER_SERVO_PIN
    );
    Serial.println(
        "Max requested Wi-Fi TX power: 19.5 dBm"
    );
    Serial.println("========================================");
 
    // Make sure camera starts OFF.
    sendCameraUART("CAM_OFF");
}
 
// ============================================================
// LOOP
// ============================================================
 
void loop()
{
    server.handleClient();
    webSocket.loop();
 
    // Camera UART receive.
    processCameraUART();
 
    // Camera command retry.
    serviceCameraRetry();
 
    // Camera auto recovery.
    serviceCameraAutoRecovery();
 
    // Movement failsafe.
    if (
        currentCommand != 'S' &&
        millis() - lastControlMillis > CONTROL_TIMEOUT
    ) {
        stopAll();
    }
 
    // Camera link timeout status recovery.
    if (
        !cameraLinkOnline() &&
        cameraDesiredOn &&
        !cameraCommandPending
    ) {
        if (cameraActualOn) {
            cameraActualOn = false;
        }
 
        copyText(
            cameraStatus,
            sizeof(cameraStatus),
            "NO LINK"
        );
 
        copyText(
            detectionText,
            sizeof(detectionText),
            "NO LINK"
        );
 
        detectionConfidence = 0.0f;
    }
 
    // Push status to browser instead of HTTP polling.
    if (
        millis() - lastStatusBroadcast >=
        STATUS_INTERVAL
    ) {
        lastStatusBroadcast = millis();
        broadcastStatus();
    }
}