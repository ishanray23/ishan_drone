#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x8c, 0x4f, 0x00, 0x35, 0xce, 0x98};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int x;
  int y;
  int z;
  int a;
  bool m;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {

  Serial.begin(115200);

  wifiSetup();
  
}
 
void loop() {

  int X = analogRead(32);
  int Y = analogRead(33);
  int Z = analogRead(34);
  int A = analogRead(35);
  int M = digitalRead(15);
  if (M == HIGH) {
    myData.m = true;
  } else {
    myData.m = false;
  }
  myData.x = map(X, 0, 4095, 0, 3);
  myData.y = map(Y, 0, 4095, 0, 3);
  myData.z = map(Z, 0, 4095, 0, 3);
  myData.a = map(A, 0, 4095, 0, 60);

  Serial.println(myData.x);
  Serial.println(myData.y);
  Serial.println(myData.z);
  Serial.println(myData.a);
  Serial.println(myData.m);
  Serial.println();
  
  sendData();

}

void sendData() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
}

void wifiSetup() {

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
   
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}
