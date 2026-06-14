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

void initJoints(bool debug, bool start_J1, bool start_J2, bool start_J3, bool start_J4, bool start_J5, bool start_J6)
{
    Serial.println("Initializing and starting requested motors...");
    uint8_t result;


    // ============== J1 =================
    J1_driver.begin();
    J1_driver.blank_time(20);
    J1_driver.en_spreadCycle(false);
    J1_driver.TCOOLTHRS(0);
    J1_driver.microsteps(Constants::Config::J1_MICOSTEPS);
    J1_driver.intpol(true);
    J1_driver.I_scale_analog(0);
    J1_driver.rms_current(Constants::Config::J1_CURRENT, Constants::Config::J1_HOLD_MULTIPLIER);
    J1_driver.TPWMTHRS(Constants::Config::J1_TPWMTHRS);


    Serial.println(F("\nTesting driver 1 connection... "));
    result= J1_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }

        // abort();
    }
    else
    {
        Serial.println("Driver 1 Initialized! ✅");
    }

    if(start_J1)
    {
        J1_driver.toff(5);
    }
    else
    {
        J1_driver.toff(0);
    }

    delay(50);
    

    // ============== J2 =================
    J2_driver.begin();
    J2_driver.blank_time(20);
    J2_driver.en_spreadCycle(false);
    J2_driver.TCOOLTHRS(0);
    J2_driver.microsteps(Constants::Config::J2_MICOSTEPS);
    J2_driver.intpol(true);
    J2_driver.I_scale_analog(0);
    J2_driver.rms_current(Constants::Config::J2_CURRENT, Constants::Config::J2_HOLD_MULTIPLIER);
    J2_driver.TPWMTHRS(Constants::Config::J2_TPWMTHRS);
    

    Serial.println(F("\nTesting driver 2 connection... "));
    result= J2_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }
        // abort();
    }
    else
    {
        Serial.println("Driver 2 Initialized! ✅");
    }

    if(start_J2)
    {
        J2_driver.toff(2);
    }
    else
    {
        J2_driver.toff(0);
    }

    delay(50);

    // ============== J3 =================
    J3_driver.begin();
    J3_driver.blank_time(20);
    J3_driver.en_spreadCycle(false);
    J3_driver.TCOOLTHRS(0);
    J3_driver.microsteps(Constants::Config::J3_MICOSTEPS);
    J3_driver.intpol(true);
    J3_driver.I_scale_analog(0);
    J3_driver.rms_current(Constants::Config::J3_CURRENT, Constants::Config::J3_HOLD_MULTIPLIER);
    J3_driver.TPWMTHRS(Constants::Config::J3_TPWMTHRS);


    Serial.println(F("\nTesting driver 3 connection... "));
    result= J3_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }

        // abort();
    }
    else
    {
        Serial.println("Driver 3 Initialized! ✅");
    }

    if(start_J3)
    {
        J3_driver.toff(2);
    }
    else
    {
        J3_driver.toff(0);
    }

    delay(50);

    // ============== J4 =================
    J4_driver.begin();
    J4_driver.blank_time(20);
    J4_driver.en_spreadCycle(false);
    J4_driver.TCOOLTHRS(0);
    J4_driver.microsteps(Constants::Config::J4_MICOSTEPS);
    J4_driver.intpol(true);
    J4_driver.I_scale_analog(0);
    J4_driver.rms_current(Constants::Config::J4_CURRENT, Constants::Config::J4_HOLD_MULTIPLIER);
    J4_driver.TPWMTHRS(Constants::Config::J4_TPWMTHRS);

    Serial.println(F("\nTesting driver 4 connection... "));
    result= J4_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }

        // abort();
    }
    else
    {
        Serial.println("Driver 4 Initialized! ✅");
    }

    if(start_J4)
    {
        J4_driver.toff(2);
    }
    else
    {
        J4_driver.toff(0);
    }

    delay(50);

    // ============== J5 =================
    J5_driver.begin();
    J5_driver.blank_time(20);
    J5_driver.en_spreadCycle(false);
    J5_driver.TCOOLTHRS(0);
    J5_driver.microsteps(Constants::Config::J5_MICOSTEPS);
    J5_driver.intpol(true);
    J5_driver.I_scale_analog(0);
    J5_driver.rms_current(Constants::Config::J5_CURRENT, Constants::Config::J5_HOLD_MULTIPLIER);
    J5_driver.TPWMTHRS(Constants::Config::J5_TPWMTHRS);

    Serial.println(F("\nTesting driver 5 connection... "));
    result= J5_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }

        // abort();
    }
    else
    {
        Serial.println("Driver 5 Initialized! ✅");
    }

    if(start_J5)
    {
        J5_driver.toff(2);
    }
    else
    {
        J5_driver.toff(0);
    }

    delay(50);

    // ============== J6 =================
    J6_driver.begin();
    J6_driver.blank_time(20);
    J6_driver.en_spreadCycle(false);
    J6_driver.TCOOLTHRS(0);
    J6_driver.microsteps(Constants::Config::J6_MICOSTEPS);
    J6_driver.intpol(true);
    J6_driver.I_scale_analog(0);
    J6_driver.rms_current(Constants::Config::J6_CURRENT, Constants::Config::J6_HOLD_MULTIPLIER);
    J6_driver.TPWMTHRS(Constants::Config::J6_TPWMTHRS);

    Serial.println(F("\nTesting driver 6 connection... "));
    result= J6_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }

        // abort();
    }
    else
    {
        Serial.println("Driver 6 Initialized! ✅");
    }

    if(start_J6)
    {
        J6_driver.toff(2);
    }
    else
    {
        J6_driver.toff(0);
    }

}