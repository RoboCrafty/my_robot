// #include <esp_now.h>
// #include <WiFi.h>
// #include <ESP32Servo.h> 

// // Define Servo 
// Servo gripperServo;
// const int servoPin = 18; // Change this to the pin your servo signal wire is connected to

// typedef struct struct_message {
//     uint8_t gripper_pos; // 0 to 140
// } struct_message;

// struct_message gripperData;

// // Callback function that executes when data is received
// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//     memcpy(&gripperData, incomingData, sizeof(gripperData));
    
//     Serial.print("Received Gripper Pos: ");
//     Serial.println(gripperData.gripper_pos);
    
//     // Command the servo to the received angle
//     gripperServo.write(gripperData.gripper_pos);
// }

// void setup() {
//     Serial.begin(115200);
    
//     // Attach the servo
//     gripperServo.attach(servoPin);
    
//     // Initialize ESP-NOW
//     WiFi.mode(WIFI_STA);
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("Error initializing ESP-NOW");
//         return;
//     }
    
//     // Register the receive callback
//     esp_now_register_recv_cb(OnDataRecv);
// }

// void loop() {
//     // ESP-NOW runs asynchronously in the background. 
//     // You can leave the loop empty or do other tasks here.
// }