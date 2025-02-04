#include <ESP32Servo.h>

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

#include <esp_now.h>
#include <WiFi.h>

int speed[] = {0,0,0,0};
int pitch;
int roll;
int yaw;
int acc;
bool b = false;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int x;
    int y;
    int z;
    int a;
    bool m;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  roll = myData.x;
  pitch = myData.y;
  yaw = myData.z;
  acc = myData.a;

  for (int i = 0; i < 4; i++) {
    speed[i] = acc;
  }

  if (acc >= 10) {
    checkX();
    checkY();
    checkZ();
  }

  checkM(myData.m);

  ESC1.write(speed[0]);
  ESC2.write(speed[1]);
  ESC3.write(speed[2]);
  ESC4.write(speed[3]);

  delay(50);

}
 
void setup() {

  Serial.begin(9600);

  ESC1.attach(32, 1000, 2000);
  ESC2.attach(33, 1000, 2000);
  ESC3.attach(26, 1000, 2000);
  ESC4.attach(27, 1000, 2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  pinMode(13, OUTPUT);

  delay(3000);
  
  wifiSetup();

}
 
void loop() {

}

void wifiSetup() {

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

}

void checkX() {
  if (roll == 0) {
    speed[2] = speed[2] - 5;
    speed[3] = speed[3] - 5;
  } else if (roll == 3) {
    speed[0] = speed[0] - 5;
    speed[1] = speed[1] - 5;
  }
}

void checkY() {
  if (pitch == 0) {
    speed[0] = speed[0] - 5;
    speed[2] = speed[2] - 5;
  } else if (pitch == 3) {
    speed[1] = speed[1] - 5;
    speed[3] = speed[3] - 5;
  }
}

void checkZ() {
  if ((pitch == 1 || pitch == 2) && (roll == 1 || roll == 2)) {
  if (yaw == 0) {
      speed[0] = speed[0] - 10;
      speed[3] = speed[3] - 10;
    } else if (yaw == 3) {
      speed[1] = speed[1] - 10;
      speed[2] = speed[2] - 10;
    }
  }
}

void checkM(bool m) {
  if (m) {
    b = !(b);
  }
  if (b) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}
