#include "motor_init.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

TMC2209Stepper J1_driver(&SERIAL_PORT2, R_SENSE, J1_ADDRESS);
TMC2209Stepper J2_driver(&SERIAL_PORT1, R_SENSE, J2_ADDRESS);
TMC2209Stepper J3_driver(&SERIAL_PORT2, R_SENSE, J3_ADDRESS);
TMC2209Stepper J4_driver(&SERIAL_PORT2, R_SENSE, J4_ADDRESS);
TMC2209Stepper J5_driver(&SERIAL_PORT1, R_SENSE, J5_ADDRESS);
TMC2209Stepper J6_driver(&SERIAL_PORT2, R_SENSE, J6_ADDRESS);

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
    J1_driver.microsteps(8);
    J1_driver.I_scale_analog(0);
    J1_driver.rms_current(1700,0.5);

    delay(50);

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
        Serial.println(F("Fix the problem and reset board."));
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
    J2_driver.microsteps(8);
    J2_driver.I_scale_analog(0);
    J2_driver.rms_current(1800,1.0);
    J2_driver.intpol(true);

    delay(50);

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
        Serial.println(F("Fix the problem and reset board."));
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
    J3_driver.microsteps(8);
    J3_driver.I_scale_analog(0);
    J3_driver.rms_current(600,1.0);

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
        Serial.println(F("Fix the problem and reset board."));
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
    J4_driver.microsteps(8);
    J4_driver.I_scale_analog(0);
    J4_driver.rms_current(1500,0.5);

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
        Serial.println(F("Fix the problem and reset board."));
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
    J5_driver.microsteps(8);
    J5_driver.I_scale_analog(0);
    J5_driver.rms_current(1500,0.5);

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
        Serial.println(F("Fix the problem and reset board."));
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
    // J6_driver.TPWMTHRS(100);
    J6_driver.TCOOLTHRS(0);
    J6_driver.microsteps(8);
    J6_driver.I_scale_analog(0);
    J6_driver.rms_current(950,0.5); 

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
        Serial.println(F("Fix the problem and reset board."));
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