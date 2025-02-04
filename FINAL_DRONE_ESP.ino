
/*#include <WiFi.h>
#include <esp_camera.h>

// Pins for Serial communication with Nano
#define RXD2 16  // Connect to Nano TX
#define TXD2 17  // Connect to Nano RX

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// HTML interface
const char* webPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Drone Control</title>
    <style>
        body { text-align: center; font-family: Arial, sans-serif; background-color: #222; color: white; }
        button { width: 80px; height: 50px; font-size: 18px; margin: 5px; }
        .controls { display: flex; flex-wrap: wrap; justify-content: center; margin-top: 10px; }
        .video { margin-top: 10px; }
    </style>
    <script>
        function sendCommand(cmd) {
            fetch('/command?cmd=' + cmd);
        }
    </script>
</head>
<body>
    <h2>Drone Control Panel</h2>
    <img class="video" src="/stream" width="320" height="240">
    <div class="controls">
        <button onclick="sendCommand('U')">Up</button>
        <button onclick="sendCommand('D')">Down</button><br>
        <button onclick="sendCommand('L')">Roll Left</button>
        <button onclick="sendCommand('R')">Roll Right</button><br>
        <button onclick="sendCommand('Y')">Yaw Left</button>
        <button onclick="sendCommand('Z')">Yaw Right</button><br>
        <button onclick="sendCommand('S')">Solenoid On</button>
        <button onclick="sendCommand('X')">Solenoid Off</button>
    </div>
</body>
</html>
)rawliteral";

// WiFi server
WiFiServer server(80);

void startCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_xclk = 0;
    config.pin_pclk = 22;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_sscb_sda = 26;
    config.pin_sscb_scl = 27;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.println("Camera init failed");
        return;
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Communication with Nano

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    startCamera();
    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        String request = client.readStringUntil('\r');
        client.flush();

        if (request.indexOf("GET /command?cmd=") != -1) {
            char command = request.charAt(18);
            Serial2.write(command);  // Send command to Nano
        }

        if (request.indexOf("GET /stream") != -1) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
            while (client.connected()) {
                camera_fb_t* fb = esp_camera_fb_get();
                if (!fb) break;
                client.println("--frame");
                client.println("Content-Type: image/jpeg");
                client.println("Content-Length: " + String(fb->len));
                client.write((const char*)fb->buf, fb->len);
                esp_camera_fb_return(fb);
                delay(100);
            }
        } else {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println();
            client.println(webPage);
        }
        client.stop();
    }
}
*/
/*
#include <WiFi.h>
#include "esp_camera.h"

const char* ssid = "LindsethWiFi";
const char* password = "8324370601";

WiFiServer server(80);

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // Communicate with ESP32-D

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    
    esp32cam::Config cfg;
    cfg.setPins(esp32cam::pins::AiThinker);
    cfg.setResolution(esp32cam::Resolution::QVGA);
    cfg.setBufferCount(2);
    cfg.setJpeg(90);
    
    if (!esp32cam::Camera.begin(cfg)) {
        Serial.println("Camera failed!");
        while (1);
    }
    
    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        String request = client.readStringUntil('\r');
        client.flush();
        
        if (request.indexOf("/UP") != -1) Serial2.println("UP");
        if (request.indexOf("/DOWN") != -1) Serial2.println("DOWN");
        if (request.indexOf("/SPIN_LEFT") != -1) Serial2.println("SPIN_LEFT");
        if (request.indexOf("/SPIN_RIGHT") != -1) Serial2.println("SPIN_RIGHT");
        if (request.indexOf("/CHARGE_SOLENOID") != -1) Serial2.println("CHARGE_SOLENOID");
        if (request.indexOf("/DISCHARGE_SOLENOID") != -1) Serial2.println("DISCHARGE_SOLENOID");

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        client.println("<html><head><title>Drone Control</title></head>");
        client.println("<body style='text-align:center;'>");
        client.println("<h2>ESP32-CAM Drone Controller</h2>");
        client.println("<img src='/stream' width='320' height='240'><br><br>");
        client.println("<a href='/UP'><button>UP</button></a>");
        client.println("<a href='/DOWN'><button>DOWN</button></a><br>");
        client.println("<a href='/SPIN_LEFT'><button>SPIN LEFT</button></a>");
        client.println("<a href='/SPIN_RIGHT'><button>SPIN RIGHT</button></a><br>");
        client.println("<a href='/CHARGE_SOLENOID'><button>CHARGE SOLENOID</button></a>");
        client.println("<a href='/DISCHARGE_SOLENOID'><button>DISCHARGE SOLENOID</button></a>");
        client.println("</body></html>");

        client.stop();
    }
}
*/
#include <WiFi.h>
#include "esp_camera.h"

const char* ssid = "LindsethWiFi";
const char* password = "8324370601";

WiFiServer server(80);

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // Communicate with ESP32-D

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_xclk = 0;
    config.pin_pclk = 22;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_sscb_sda = 26;
    config.pin_sscb_scl = 27;
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Init with high specs to pre-allocate larger buffers
    if (psramFound()) {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 10;  //0-63 lower means higher quality
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }
    
    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
    
    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        String request = client.readStringUntil('\r');
        client.flush();
        
        if (request.indexOf("/UP") != -1) Serial2.println("UP");
        if (request.indexOf("/DOWN") != -1) Serial2.println("DOWN");
        if (request.indexOf("/SPIN_LEFT") != -1) Serial2.println("SPIN_LEFT");
        if (request.indexOf("/SPIN_RIGHT") != -1) Serial2.println("SPIN_RIGHT");
        if (request.indexOf("/CHARGE_SOLENOID") != -1) Serial2.println("CHARGE_SOLENOID");
        if (request.indexOf("/DISCHARGE_SOLENOID") != -1) Serial2.println("DISCHARGE_SOLENOID");

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        client.println("<html><head><title>Drone Control</title></head>");
        client.println("<body style='text-align:center;'>");
        client.println("<h2>ESP32-CAM Drone Controller</h2>");
        client.println("<img src='/stream' width='320' height='240'><br><br>");
        client.println("<a href='/UP'><button>UP</button></a>");
        client.println("<a href='/DOWN'><button>DOWN</button></a><br>");
        client.println("<a href='/SPIN_LEFT'><button>SPIN LEFT</button></a>");
        client.println("<a href='/SPIN_RIGHT'><button>SPIN RIGHT</button></a><br>");
        client.println("<a href='/CHARGE_SOLENOID'><button>CHARGE SOLENOID</button></a>");
        client.println("<a href='/DISCHARGE_SOLENOID'><button>DISCHARGE SOLENOID</button></a>");
        client.println("</body></html>");

        client.stop();
    }
}
