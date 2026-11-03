#include <esp_now.h>
#include <WiFi.h>
#include <SCServo.h>

SMS_STS st;

const int RX_PIN = 20;
const int TX_PIN = 21;
const uint8_t SERVO_ID = 10;

const int OPEN_STEP   = 3754;  // 330 deg -> gripper_pos 0
const int CLOSED_STEP = 2161;  // 190 deg -> gripper_pos 140

const int SPEED   = 3000;
const uint8_t ACC = 254;

typedef struct struct_message {
    uint8_t gripper_pos; // 0 (open) to 140 (closed)
} struct_message;

struct_message gripperData;

volatile int targetPos = OPEN_STEP;
volatile bool newTarget = false;

int gripperToStep(uint8_t g) {
    if (g > 140) g = 140;
    return OPEN_STEP - (int)((float)g * 4096.0f / 360.0f);
}

// Only store the target here. The serial write happens in loop().
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(gripperData)) return;
    memcpy(&gripperData, incomingData, sizeof(gripperData));
    targetPos = gripperToStep(gripperData.gripper_pos);
    newTarget = true;
}

void setup() {
    Serial.begin(115200);

    Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
    st.pSerial = &Serial1;
    delay(500);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    st.WritePosEx(SERVO_ID, OPEN_STEP, SPEED, ACC);
}

void loop() {
    if (newTarget) {
        newTarget = false;
        int pos = targetPos;
        st.WritePosEx(SERVO_ID, pos, SPEED, ACC);
        Serial.printf("gripper %u -> step %d\n", gripperData.gripper_pos, pos);
    }
}
