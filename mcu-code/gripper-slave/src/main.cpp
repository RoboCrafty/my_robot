// #include <Arduino.h>

// void setup() {
//   Serial.begin(115200);
//   delay(2000); 
  
//   // Read the burned-in MAC address directly from the silicon
//   uint64_t chipid = ESP.getEfuseMac(); 
  
//   uint8_t mac[6];
//   for(int i = 0; i < 6; i++) {
//     mac[i] = (chipid >> (8 * i)) & 0xFF;
//   }
  
//   Serial.print("Hardware eFuse MAC: ");
//   Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", 
//                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
// }

// void loop() {
// }


// #include <ESP32Servo.h>

// const int SERVO_PIN = 19;
// Servo gripperServo;

// // Default break-in positions
// int posA = 0;
// int posB = 140;

// unsigned long lastMoveTime = 0;
// bool isAtPosA = true;

// void setup() {
//     Serial.begin(115200);
    
//     ESP32PWM::allocateTimer(0);
//     ESP32PWM::allocateTimer(1);
//     ESP32PWM::allocateTimer(2);
//     ESP32PWM::allocateTimer(3);
    
//     gripperServo.setPeriodHertz(50); 
//     gripperServo.attach(SERVO_PIN, 500, 2500); 
    
//     Serial.println("\n--- Gripper Break-In Routine ---");
//     Serial.printf("Currently sweeping between %d and %d degrees.\n", posA, posB);
//     Serial.println("To change limits, type two numbers separated by a comma (e.g., '10,130') and press ENTER.");
// }

// void loop() {
//     // 1. Check for new commands from the terminal
//     if (Serial.available() > 0) {
//         String input = Serial.readStringUntil('\n');
//         input.trim();
        
//         int commaIndex = input.indexOf(',');
//         if (commaIndex > 0) {
//             posA = input.substring(0, commaIndex).toInt();
//             posB = input.substring(commaIndex + 1).toInt();
            
//             // Constrain to standard servo angles
//             posA = constrain(posA, 0, 180);
//             posB = constrain(posB, 0, 180);
            
//             Serial.printf("New limits set! Sweeping between %d and %d.\n", posA, posB);
//         } else if (input.length() > 0) {
//             Serial.println("Invalid format. Please use a comma, e.g., '0,140'");
//         }
//     }

//     // 2. Automatically sweep every 1000 milliseconds (1 second)
//     if (millis() - lastMoveTime >= 1000) {
//         lastMoveTime = millis();
        
//         if (isAtPosA) {
//             gripperServo.write(posB);
//             isAtPosA = false;
//         } else {
//             gripperServo.write(posA);
//             isAtPosA = true;
//         }
//     }
// }








// #include <ESP32Servo.h>

// const int SERVO_PIN = 19;
// Servo gripperServo;

// void setup() {
//     Serial.begin(115200);
    
//     // Attach the servo and set to a safe middle position
//     gripperServo.attach(SERVO_PIN);
//     gripperServo.write(90);
    
//     Serial.println("\n--- Gripper Limit Tester ---");
//     Serial.println("Enter an angle between 0 and 180:");
// }

// void loop() {
//     if (Serial.available() > 0) {
//         // Read the integer from the terminal
//         int angle = Serial.parseInt();
        
//         // Clear out any lingering newline characters from the buffer
//         while (Serial.available() > 0) {
//             Serial.read();
//         }

//         // Apply the movement
//         if (angle >= 0 && angle <= 180) {
//             Serial.printf("Command received. Moving to: %d degrees\n", angle);
//             gripperServo.write(angle);
//         } else {
//             Serial.println("Invalid input. Please enter a number between 0 and 180.");
//         }
//     }
// }









































#include <esp_now.h>
#include <WiFi.h>

// Slave MAC Address
uint8_t slaveAddress[] = {0x78, 0x1C, 0x3C, 0xE1, 0x08, 0x1C};

typedef struct struct_message {
    uint8_t gripper_pos; // 0 to 140
} struct_message;

struct_message gripperData;
esp_now_peer_info_t peerInfo;

// Keep track of the current angle (starts at 0)
int currentAngle = 0; 

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Delivery Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA); 
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, slaveAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("Setup Complete.");
    Serial.println("Press RIGHT ARROW (or 'd') to open (+5 deg).");
    Serial.println("Press LEFT ARROW (or 'a') to close (-5 deg).");
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        bool positionChanged = false;
        
        // 1. Check for ANSI escape sequence (Actual Arrow Keys)
        if (c == 0x1B) { // ESC character
            delay(5); // Wait a tiny bit for the rest of the sequence to arrive
            if (Serial.available() >= 2) {
                char bracket = Serial.read();
                char dir = Serial.read();
                
                if (bracket == '[') {
                    if (dir == 'C') { // Right Arrow
                        currentAngle += 5;
                        positionChanged = true;
                    } else if (dir == 'D') { // Left Arrow
                        currentAngle -= 5;
                        positionChanged = true;
                    }
                }
            }
        } 
        // 2. Fallback for standard keys (Easier for Arduino IDE Serial Monitor)
        else if (c == 'd' || c == 'D') { // 'D' for Right
            currentAngle += 5;
            positionChanged = true;
        } 
        else if (c == 'a' || c == 'A') { // 'A' for Left
            currentAngle -= 5;
            positionChanged = true;
        }

        // If a valid key was pressed, process and send
        if (positionChanged) {
            // Constrain the angle so it doesn't go below 0 or above 140
            if (currentAngle > 140) currentAngle = 140;
            if (currentAngle < 0) currentAngle = 0;

            gripperData.gripper_pos = currentAngle;
            
            Serial.print("Moved by 5 degrees. New Position: ");
            Serial.println(currentAngle);
            
            esp_now_send(slaveAddress, (uint8_t *) &gripperData, sizeof(gripperData));
            
            // Clear out any remaining characters (like 'Enter' key \n or \r)
            while(Serial.available() > 0) {
                Serial.read();
            }
        }
    }
}