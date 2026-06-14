#pragma once
#include <ruckig/ruckig.hpp>


using namespace ruckig;
const int DOFs = 1;
inline Ruckig<DOFs> ruck(0.001);
inline InputParameter<DOFs> input;
inline OutputParameter<DOFs> output;


inline void setupRuckig() 
{
    // HARD CEILING for moveTimed on J6: MIN_CMD_TICKS = TICKS_PER_S/5000 = 3200
    // ticks/step => max 5000 steps/s => 5000/44.44 = 112.5 deg/s = 1.96 rad/s.
    // Anything above that makes moveTimed reject windows with ErrorTicksTooLow.
    // Keep ~30% margin below the ceiling.
    input.max_velocity     = {7.0};    // rad/s  (J6 ceiling ~1.96 rad/s)
    input.max_acceleration = {30.0};    // rad/s²
    input.max_jerk         = {120.0};   // rad/s³

    input.current_position = {0.0};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};

    // 3600 degrees / 360 = 10 Revolutions!
    input.target_position = {0.0};
    input.target_velocity = {0.0}; 
    input.target_acceleration = {0.0};
    // input.synchronization = ruckig::Synchronization::Phase;

    // // Validation Check...
    // try {
    //     ruck.validate_input(input);
    //     Serial.println("Ruckig Input Validation: SUCCESS");
    // } catch (const std::exception& e) {
    //     Serial.print("FATAL RUCKIG ERROR: ");
    //     Serial.println(e.what());
    //     while (true) delay(1000);
    // }
}
void handleRuckigTargetUpdate(float dist) {
    input.target_position   = {dist}; 
}