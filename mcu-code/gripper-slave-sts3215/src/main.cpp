#include <esp_now.h>
#include <WiFi.h>
#include <SCServo.h>

// TX and RX share one wire, so every transmitted byte is also received; drop it before the reply is parsed.
const bool BUS_ECHO = true;

class EchoSTS : public SMS_STS {
    size_t sent = 0;
protected:
    int writeSCS(unsigned char *d, int n) override { sent += n; return SMS_STS::writeSCS(d, n); }
    int writeSCS(unsigned char b) override { sent += 1; return SMS_STS::writeSCS(b); }
    void rFlushSCS() override { sent = 0; SMS_STS::rFlushSCS(); }
    void wFlushSCS() override {
        if (BUS_ECHO) {
            pSerial->flush();
            uint32_t t0 = micros();
            while (sent && micros() - t0 < 1000) {
                if (pSerial->read() != -1) sent--;
            }
        }
        sent = 0;
    }
};

EchoSTS st;

const int RX_PIN = 20;
const int TX_PIN = 21;
const uint8_t SERVO_ID = 10;

const int OPEN_STEP   = 3600;  // 330 deg -> gripper_pos 0
const int CLOSED_STEP = 2000;  // 190 deg -> gripper_pos 140
const int OPEN_LIMIT_STEP = 3660;  // jaws stop mechanically near 3665; aiming past it only pushes the gears into the stopper

const int SPEED   = 3000;  // steps/s; the field is a 15-bit magnitude, so values above 32767 set the direction flag
const uint8_t ACC = 0;  // 0 = Instant max acceleration

const uint16_t MOVE_TORQUE = 1000;  // 0-1000 (0.1% units); drive level while the jaws move freely
uint16_t gripTorque = 60;  // drive level while holding an object; send 't<n>' on the serial port to change it live
uint16_t closeTorque = 500;  // drive cap while closing, so a missed contact stays bounded; 'c<n>' changes it live

// Contact = the jaws are driven hard (LOAD_CONTACT) but not progressing. A slow start in free air has low drive.
// Stopping right after a far approach is detected quickly; otherwise the drive must persist for BLOCKED_MS.
const int      STALL_ERR       = 80;   // steps
const int      STALL_MOVE      = 6;    // steps of progress that still count as moving
const uint32_t STALL_MS        = 30;   // stopped this long after moving -> contact
const int      MOVING_STEPS    = 3;    // steps between polls that count as moving
const uint32_t MOVED_WINDOW_MS = 150;
const int      LOAD_CONTACT    = 250;  // drive level (0-1000) that means the jaws are pushing on something
const int      BLOCK_ERR       = 8;    // steps of error below which the jaws are simply at their target
const uint32_t BLOCKED_MS      = 200;  // no progress this long while pushing -> contact
const int      STALL_SPEED     = 120;  // steps/s of creep that still counts as blocked (a compliant object gives a little)
const int      LOAD_HOT_PCT    = 80;   // percent of the closing cap that must not persist while closing, even if the jaws creep
const uint32_t HOT_MS          = 150;
const int      HOT_MOVE        = 40;   // steps of progress within HOT_MS that mean the jaws are travelling, not stuck
const int      TARGET_SAME     = 10;   // steps the target may drift and still count as the same move
const int      GRIP_PUSH       = 15;   // steps past the contact point to aim for while holding
const int      RELEASE_ERR     = 25;   // below this the target no longer presses past the jaws
const uint32_t POLL_MS        = 10;

// Register Addresses for STS Servos
#define SMS_STS_P_GAIN 21  // 0x15
#define SMS_STS_D_GAIN 22  // 0x16
#define SMS_STS_I_GAIN 23  // 0x17
#define SMS_STS_PUNCH  24  // 0x18 (2 bytes: 24 & 25)

// Factory values for this servo: P 32, D 32, I 0, punch 16, dead zones 1.
const uint8_t  GAIN_P    = 48;
const uint8_t  GAIN_D    = 48;
const uint16_t PUNCH     = 16;  // minimum drive for any error outside the dead zone; large values make it hunt
const uint8_t  DEAD_ZONE = 2;   // steps

typedef struct struct_message {
    uint8_t gripper_pos; // 0 (open) to 140 (closed)
} struct_message;

struct_message gripperData;

volatile int targetPos = OPEN_LIMIT_STEP;
volatile bool newTarget = false;
volatile uint32_t rxCount = 0;
int lastSentPos = -1;

uint16_t torque = 0;
bool holding = false;
int holdDir = 0;        // sign of (target - position) when the stall was detected
bool stalling = false;
bool stallAfterMotion = false;
uint32_t stallSince = 0;
int stallPos = 0;
uint32_t hotSince = 0;
int hotPos = 0;
int prevPos = OPEN_STEP;
uint32_t lastMoveMs = 0;
int load = 0;
int current = 0;  // raw servo units
int peakCurrent = 0;  // max since the last status line
int requestedPos = OPEN_LIMIT_STEP;  // latest target from the sender, even while holding
int holdPos = 0;
int lastMoveErr = 0;
int lastMoveTarget = 0;
uint32_t lastPoll = 0;
uint32_t pollInterval = POLL_MS;
uint32_t fbOk = 0, fbFail = 0;
int measuredPos = OPEN_STEP;

int gripperToStep(uint8_t g) {
    if (g > 140) g = 140;
    int step = OPEN_STEP - (int)((float)g * 4096.0f / 360.0f);
    return constrain(step, CLOSED_STEP, OPEN_LIMIT_STEP);
}

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
    if (len != sizeof(gripperData)) return;
    memcpy(&gripperData, incomingData, sizeof(gripperData));
    targetPos = gripperToStep(gripperData.gripper_pos);
    newTarget = true;
    rxCount++;
}

// EEPROM has limited write cycles, so only registers that differ are written.
void setupPIDGains(uint8_t id) {
    struct Setting { uint8_t addr; bool word; uint16_t value; };
    const Setting settings[] = {
        {SMS_STS_P_GAIN, false, GAIN_P},
        {SMS_STS_D_GAIN, false, GAIN_D},
        {SMS_STS_PUNCH, true, PUNCH},
        {SMS_STS_CW_DEAD, false, DEAD_ZONE},
        {SMS_STS_CCW_DEAD, false, DEAD_ZONE},
    };
    bool unlocked = false;
    for (const Setting &s : settings) {
        int current = s.word ? st.readWord(id, s.addr) : st.readByte(id, s.addr);
        if (current == s.value) continue;
        if (!unlocked) {
            st.unLockEprom(id);
            unlocked = true;
        }
        if (s.word) st.writeWord(id, s.addr, s.value);
        else st.writeByte(id, s.addr, s.value);
    }
    if (unlocked) st.LockEprom(id);
}

const uint8_t NEW_SERVO_ID = 1;  // factory ID of a fresh servo

// Names follow Feetech's STS memory table; the raw addresses and values are what matter for restoring.
struct Reg { uint8_t addr, len; const char *name; };
const Reg REGS[] = {
    {0, 1, "Firmware major"}, {1, 1, "Firmware minor"}, {3, 2, "Model"},
    {5, 1, "ID"}, {6, 1, "Baud rate"}, {7, 1, "Return delay"}, {8, 1, "Response level"},
    {9, 2, "Min angle limit"}, {11, 2, "Max angle limit"},
    {13, 1, "Max temperature"}, {14, 1, "Max voltage"}, {15, 1, "Min voltage"},
    {16, 2, "Max torque limit"}, {18, 1, "Phase"}, {19, 1, "Unloading condition"}, {20, 1, "LED alarm condition"},
    {21, 1, "P gain"}, {22, 1, "D gain"}, {23, 1, "I gain"}, {24, 2, "Min startup force (punch)"},
    {26, 1, "CW dead zone"}, {27, 1, "CCW dead zone"}, {28, 2, "Protection current"},
    {30, 1, "Angular resolution"}, {31, 2, "Position offset"}, {33, 1, "Operation mode"},
    {34, 1, "Protection torque"}, {35, 1, "Protection time"}, {36, 1, "Overload torque"},
    {37, 1, "Speed loop P"}, {38, 1, "Over-current time"}, {39, 1, "Speed loop I"},
    {40, 1, "Torque enable (SRAM)"}, {41, 1, "Acceleration (SRAM)"}, {42, 2, "Goal position (SRAM)"},
    {44, 2, "Goal time (SRAM)"}, {46, 2, "Goal speed (SRAM)"}, {48, 2, "Torque limit (SRAM)"},
    {55, 1, "Lock (SRAM)"},
};

uint8_t snapshot[56];  // addresses 0..55, read before this firmware writes anything
bool snapshotOk = false;

int findServo() {
    int id = st.Ping(SERVO_ID);
    if (id < 0) id = st.Ping(NEW_SERVO_ID);
    if (id < 0) id = st.Ping(0xfe);  // broadcast: answers with its own ID when a single servo is on the bus
    return id;
}

bool assignServoId() {
    int id = findServo();
    if (id < 0) {
        Serial.println("No servo answered on the bus");
        return false;
    }
    if (id == SERVO_ID) return true;

    Serial.printf("Servo found at ID %d, changing it to %d\n", id, SERVO_ID);
    st.unLockEprom(id);
    st.writeByte(id, SMS_STS_ID, SERVO_ID);
    st.LockEprom(SERVO_ID);
    delay(100);
    if (st.Ping(SERVO_ID) != SERVO_ID) {
        Serial.println("ID change failed");
        return false;
    }
    return true;
}

bool readSnapshot() {
    for (uint8_t a = 0; a < sizeof(snapshot); a += 8) {
        if (st.Read(SERVO_ID, a, snapshot + a, 8) != 8) return false;
    }
    return true;
}

void printSnapshot() {
    Serial.printf("=== Servo ID %d registers, read at boot before any change (send 'd' to reprint) ===\n", SERVO_ID);
    for (const Reg &r : REGS) {
        unsigned v = snapshot[r.addr];
        if (r.len == 2) v |= snapshot[r.addr + 1] << 8;
        Serial.printf("%2u  %-26s %u\n", r.addr, r.name, v);
    }
    Serial.print("EEPROM raw [0..39] = {");
    for (int i = 0; i < 40; i++) Serial.printf("%u%s", snapshot[i], i < 39 ? ", " : "}\n");
}

// SRAM register: resets whenever the servo loses power.
void setTorque(uint16_t t) {
    if (t == torque) return;
    st.writeWord(SERVO_ID, SMS_STS_TORQUE_LIMIT_L, t);
    torque = t;
}

void sendTarget(int pos) {
    setTorque(pos < measuredPos - BLOCK_ERR ? closeTorque : MOVE_TORQUE);
    st.WritePosEx(SERVO_ID, pos, SPEED, ACC);
    lastSentPos = pos;
}

void enterHold(int err) {
    holding = true;
    holdDir = err > 0 ? 1 : -1;
    holdPos = measuredPos;
    stalling = false;
    hotSince = 0;
    setTorque(gripTorque);
    // Aim just past the contact point so the push is proportional to a small error instead of saturating the drive.
    st.WritePosEx(SERVO_ID, constrain(holdPos + holdDir * GRIP_PUSH, CLOSED_STEP, OPEN_LIMIT_STEP), SPEED, ACC);
    Serial.printf("hold pos=%d target=%d load=%d current=%d\n", measuredPos, requestedPos, load, current);
}

void releaseHold() {
    holding = false;
    stalling = false;
    sendTarget(requestedPos);
    Serial.println("release");
}

void pollStall() {
    uint32_t now = millis();
    if (now - lastPoll < pollInterval) return;
    lastPoll = now;

    if (st.FeedBack(SERVO_ID) < 0) {
        fbFail++;
        pollInterval = 250;  // each failed read blocks up to IOTimeOut
        return;
    }
    fbOk++;
    pollInterval = POLL_MS;
    measuredPos = st.ReadPos(-1);
    load = st.ReadLoad(-1);
    current = st.ReadCurrent(-1);
    int err = lastSentPos - measuredPos;

    if (abs(current) > peakCurrent) peakCurrent = abs(current);

    if (abs(measuredPos - prevPos) >= MOVING_STEPS) {
        lastMoveMs = now;
        lastMoveErr = abs(err);
        lastMoveTarget = lastSentPos;
    }
    prevPos = measuredPos;

    if (holding) {
        if ((requestedPos - measuredPos) * holdDir < RELEASE_ERR) releaseHold();
        return;
    }

    bool hot = err < -BLOCK_ERR && abs(load) >= closeTorque * LOAD_HOT_PCT / 100;
    if (hot && hotSince && abs(measuredPos - hotPos) > HOT_MOVE) hotSince = 0;  // still travelling
    if (!hot) {
        hotSince = 0;
    } else if (!hotSince) {
        hotSince = now;
        hotPos = measuredPos;
    } else if (now - hotSince >= HOT_MS) {
        enterHold(err);
        return;
    }

    // Only closing can be blocked by an object; the slow start when reversing away from one looks like a stall.
    bool slow = !stalling || abs(measuredPos - stallPos) <= STALL_MOVE + (int)(now - stallSince) * STALL_SPEED / 1000;
    if (err < -BLOCK_ERR && slow) {
        if (!stalling) {
            stalling = true;
            stallSince = now;
            stallPos = measuredPos;
            stallAfterMotion = now - lastMoveMs < MOVED_WINDOW_MS && lastMoveErr > STALL_ERR &&
                               abs(lastSentPos - lastMoveTarget) <= TARGET_SAME;
        } else {
            uint32_t t = now - stallSince;
            bool hitFarTarget = stallAfterMotion && abs(err) > STALL_ERR && t >= STALL_MS;
            if (abs(load) >= LOAD_CONTACT && (hitFarTarget || t >= BLOCKED_MS)) enterHold(err);
        }
    } else {
        stalling = false;
    }
}

void setup() {
    Serial.begin(115200);

    Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
    st.pSerial = &Serial1;
    delay(500);

    // USB serial reconnects after reset; give the monitor a moment so the dump isn't lost.
    for (uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) delay(10);

    if (assignServoId()) {
        snapshotOk = readSnapshot();
        if (snapshotOk) printSnapshot();
        else Serial.println("Register read failed");
    }

    // Tune PID parameters for twitch-like dynamic tracking
    setupPIDGains(SERVO_ID);
    setTorque(MOVE_TORQUE);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    st.WritePosEx(SERVO_ID, OPEN_LIMIT_STEP, SPEED, ACC);
    lastSentPos = OPEN_LIMIT_STEP;
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'd' && snapshotOk) {
            printSnapshot();
        } else if (c == 't') {
            int v = Serial.parseInt();
            if (v >= 10 && v <= 1000) {
                gripTorque = v;
                if (holding) setTorque(gripTorque);
                Serial.printf("grip torque = %d\n", v);
            }
        } else if (c == 'c') {
            int v = Serial.parseInt();
            if (v >= 100 && v <= 1000) {
                closeTorque = v;
                Serial.printf("close torque = %d\n", v);
            }
        }
    }

    if (newTarget) {
        noInterrupts();
        int pos = targetPos;
        newTarget = false;
        interrupts();

        requestedPos = pos;
        if (!holding && abs(pos - lastSentPos) > 2) sendTarget(pos);
    }

    pollStall();

    static uint32_t lastStatus = 0;
    if (millis() - lastStatus >= 1000) {
        lastStatus = millis();
        Serial.printf("pos=%d target=%d torque=%u hold=%d load=%d current=%d peak=%d rx=%lu read_ok=%lu read_fail=%lu err=%d\n",
                      measuredPos, lastSentPos, torque, holding, load, current, peakCurrent, (unsigned long)rxCount,
                      (unsigned long)fbOk, (unsigned long)fbFail, st.getLastError());
        peakCurrent = 0;
    }
}