#include <WiFi.h>
#include "robot_config.h"

const char* AP_SSID = "ESP32_ROBOT";
const char* AP_PASS = "12345678";

WebServer server(80);

/* HTTP Callback Handlers */
void handle_cmd() {
    if (server.hasArg("plain")) {
        apply_command(server.arg("plain")[0]);
        server.send(200, "text/plain", build_telemetry_string());
    }
}

void handle_status() {
    server.send(200, "text/plain", build_telemetry_string());
}

void setup() {
    Serial.begin(115200);
    
    // Initialize Hardware
    robot_init();

    // Initialize WiFi
    WiFi.softAP(AP_SSID, AP_PASS);
    
    // Setup API Routes
    server.on("/api/cmd", HTTP_POST, handle_cmd);
    server.on("/api/status", HTTP_GET, handle_status);
    server.begin();

    Serial.println("System Online");
}

void loop() {
    // 1. Handle incoming Network Requests
    server.handleClient();

    // 2. Run the Robot State Machine (Ramping, Safety, Steppers)
    service_robot();
    
    // 3. Background tasks or Serial debugging could go here
}