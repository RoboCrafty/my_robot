// // #include <esp_now.h>
// #include <WiFi.h>

// // Slave MAC Address
// uint8_t slaveAddress[] = {0x78, 0x1C, 0x3C, 0xE1, 0x08, 0x1C};

// typedef struct struct_message {
//     uint8_t gripper_pos; // 0 to 140
// } struct_message;

// struct_message gripperData;
// esp_now_peer_info_t peerInfo;

// // Keep track of the current angle (starts at 0)
// int currentAngle = 0; 

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     Serial.print("Delivery Status: ");
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_STA); 
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("Error initializing ESP-NOW");
//         return;
//     }
    
//     esp_now_register_send_cb(OnDataSent);
//     memcpy(peerInfo.peer_addr, slaveAddress, 6);
//     peerInfo.channel = 0;  
//     peerInfo.encrypt = false;
    
//     if (esp_now_add_peer(&peerInfo) != ESP_OK){
//         Serial.println("Failed to add peer");
//         return;
//     }

//     Serial.println("Setup Complete.");
//     Serial.println("Press RIGHT ARROW (or 'd') to open (+5 deg).");
//     Serial.println("Press LEFT ARROW (or 'a') to close (-5 deg).");
// }

// void loop() {
//     if (Serial.available() > 0) {
//         char c = Serial.read();
//         bool positionChanged = false;
        
//         // 1. Check for ANSI escape sequence (Actual Arrow Keys)
//         if (c == 0x1B) { // ESC character
//             delay(5); // Wait a tiny bit for the rest of the sequence to arrive
//             if (Serial.available() >= 2) {
//                 char bracket = Serial.read();
//                 char dir = Serial.read();
                
//                 if (bracket == '[') {
//                     if (dir == 'C') { // Right Arrow
//                         currentAngle += 5;
//                         positionChanged = true;
//                     } else if (dir == 'D') { // Left Arrow
//                         currentAngle -= 5;
//                         positionChanged = true;
//                     }
//                 }
//             }
//         } 
//         // 2. Fallback for standard keys (Easier for Arduino IDE Serial Monitor)
//         else if (c == 'd' || c == 'D') { // 'D' for Right
//             currentAngle += 5;
//             positionChanged = true;
//         } 
//         else if (c == 'a' || c == 'A') { // 'A' for Left
//             currentAngle -= 5;
//             positionChanged = true;
//         }

//         // If a valid key was pressed, process and send
//         if (positionChanged) {
//             // Constrain the angle so it doesn't go below 0 or above 140
//             if (currentAngle > 140) currentAngle = 140;
//             if (currentAngle < 0) currentAngle = 0;

//             gripperData.gripper_pos = currentAngle;
            
//             Serial.print("Moved by 5 degrees. New Position: ");
//             Serial.println(currentAngle);
            
//             esp_now_send(slaveAddress, (uint8_t *) &gripperData, sizeof(gripperData));
            
//             // Clear out any remaining characters (like 'Enter' key \n or \r)
//             while(Serial.available() > 0) {
//                 Serial.read();
//             }
//         }
//     }
// }