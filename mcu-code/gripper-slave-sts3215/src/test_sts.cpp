// #include <SCServo.h>

// SMS_STS st;

// const int RX_PIN = 20;
// const int TX_PIN = 21;
// const uint8_t SERVO_ID = 10;

// const int  ZERO_STEP = 0;  // step value at your "closed" position
// const int  DIR       = 1;     // 1 or -1 to flip direction
// const int  SPEED     = 3000;  // steps/s (max about 3000)
// const uint8_t ACC    = 254;    // units of 100 steps/s^2 (0-254)

// void setup() {
//   Serial.begin(115200);
//   Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
//   st.pSerial = &Serial1;
//   delay(500);
//   Serial.println("Type a target angle in degrees (0 = closed) and press enter");
// }

// void loop() {
//   if (Serial.available()) {
//     float deg = Serial.readStringUntil('\n').toFloat();
//     int pos = ZERO_STEP + DIR * (int)(deg * 4096.0 / 360.0);
//     pos = constrain(pos, 0, 4095);
//     st.WritePosEx(SERVO_ID, pos, SPEED, ACC);
//     Serial.printf("angle %.1f -> step %d\n", deg, pos);
//   }
// }