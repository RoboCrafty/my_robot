#include "motor_init.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[6] = {nullptr};


TMC2209Stepper J1_driver(&SERIAL_PORT2, Constants::Driver::R_SENSE_BTT,     Constants::Driver::J1_ADDRESS);
TMC2209Stepper J2_driver(&SERIAL_PORT1, Constants::Driver::R_SENSE_BTT,     Constants::Driver::J2_ADDRESS);
TMC2209Stepper J3_driver(&SERIAL_PORT2, Constants::Driver::R_SENSE_BTT,     Constants::Driver::J3_ADDRESS);
TMC2209Stepper J4_driver(&SERIAL_PORT2, Constants::Driver::R_SENSE_Random,  Constants::Driver::J4_ADDRESS);
TMC2209Stepper J5_driver(&SERIAL_PORT1, Constants::Driver::R_SENSE_BTT,     Constants::Driver::J5_ADDRESS);
TMC2209Stepper J6_driver(&SERIAL_PORT2, Constants::Driver::R_SENSE_Random,  Constants::Driver::J6_ADDRESS);

// Layout
// ESP32 --- J5
//       --- J2
//       --- J4
//       --- J6 
//       --- J1
//       --- J3

TMC2209Stepper* const DRIVERS[6] = {&J1_driver, &J2_driver, &J3_driver, &J4_driver, &J5_driver, &J6_driver};

namespace {
struct DriverCfg {
    uint16_t current;
    float    hold_multiplier;
    uint16_t microsteps;
    uint32_t tpwmthrs;
    uint8_t  toff_on;      // chopper off-time used when the motor is energized at init
};

using namespace Constants::Config;
const DriverCfg CFG[6] = {
    {J1_CURRENT, J1_HOLD_MULTIPLIER, J1_MICOSTEPS, J1_TPWMTHRS, 5},
    {J2_CURRENT, J2_HOLD_MULTIPLIER, J2_MICOSTEPS, J2_TPWMTHRS, 2},
    {J3_CURRENT, J3_HOLD_MULTIPLIER, J3_MICOSTEPS, J3_TPWMTHRS, 2},
    {J4_CURRENT, J4_HOLD_MULTIPLIER, J4_MICOSTEPS, J4_TPWMTHRS, 2},
    {J5_CURRENT, J5_HOLD_MULTIPLIER, J5_MICOSTEPS, J5_TPWMTHRS, 2},
    {J6_CURRENT, J6_HOLD_MULTIPLIER, J6_MICOSTEPS, J6_TPWMTHRS, 2},
};

SemaphoreHandle_t tmc_mutex = nullptr;
}

void tmcLockInit()
{
    if (!tmc_mutex) tmc_mutex = xSemaphoreCreateMutex();
}

TmcLock::TmcLock()  { xSemaphoreTake(tmc_mutex, portMAX_DELAY); }
TmcLock::~TmcLock() { xSemaphoreGive(tmc_mutex); }

// Writes the full register set. TMCStepper keeps shadow copies of every
// register, so this also restores a driver that lost VM and reset to its OTP
// defaults (wrong microstepping, analog current scaling).
void configureDriver(uint8_t idx, bool energize)
{
    TMC2209Stepper& d = *DRIVERS[idx];
    const DriverCfg& c = CFG[idx];
    d.begin();
    d.blank_time(20);
    d.en_spreadCycle(false);
    d.TCOOLTHRS(0);
    d.microsteps(c.microsteps);
    d.intpol(true);
    d.I_scale_analog(0);
    d.rms_current(c.current, c.hold_multiplier);
    d.TPWMTHRS(c.tpwmthrs);
    d.toff(energize ? c.toff_on : 0);
}

uint8_t initJoints(bool debug, bool start_J1, bool start_J2, bool start_J3, bool start_J4, bool start_J5, bool start_J6)
{
    Serial.println("Initializing and starting requested motors...");
    const bool start[6] = {start_J1, start_J2, start_J3, start_J4, start_J5, start_J6};
    uint8_t ok_mask = 0;

    for (uint8_t i = 0; i < 6; i++)
    {
        configureDriver(i, start[i]);

        Serial.printf("\nTesting driver %d connection... \n", i + 1);
        uint8_t result = DRIVERS[i]->test_connection();
        if (result)
        {
            Serial.println(F("Failed! ❌"));
            Serial.print(F("Likely cause: "));
            switch (result)
            {
                case 1: Serial.println(F("Loose connection")); break;
                case 2: Serial.println(F("no power")); break;
            }
        }
        else
        {
            Serial.printf("Driver %d Initialized! ✅\n", i + 1);
            DRIVERS[i]->GSTAT(0b111);   // clear the power-up reset flag so the power monitor starts clean
            ok_mask |= (uint8_t)(1u << i);
        }
        delay(50);
    }
    return ok_mask;
}

void turnDriverOff(uint motorNo)
{
    if (motorNo >= 1 && motorNo <= 6) DRIVERS[motorNo - 1]->toff(0);
}

void turnDriverOn(uint motorNo)
{
    if (motorNo >= 1 && motorNo <= 6) DRIVERS[motorNo - 1]->toff(2);
}
