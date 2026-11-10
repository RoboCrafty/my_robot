#pragma once
#include <ruckig/ruckig.hpp>


using namespace ruckig;
const int DOFs = 1;
inline Ruckig<DOFs> ruck(0.001);
inline InputParameter<DOFs> input;
inline OutputParameter<DOFs> output;


inline void setupRuckig() 
{
    input.max_velocity = {0.50};       
    input.max_acceleration = {5.50}; 
    input.max_jerk = {10.0};         

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