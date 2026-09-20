#pragma once

#include <cmath>
#include <numbers>
namespace airbrake {

/**
    A single telemetry row using units produced by the flight computer.

    This structure represents the raw flight data before normalization into the 
    simulator's internal SI representations.
*/
struct RawTelemetrySample {
    double time_ms;
    double pressure_hpa;
    double altitude_m;
    double start_altitude_m;
    double temperature_c;
    double vertical_velocity_mps;
};

/**
Normalized vertical flight state used by the physics model.

All quantities are in SI units. Altitude is relative to the launch reference altitude,
while time remains relative to the telemetry timeline
*/
struct RocketState {
    double tilt_from_vertical_deg; //  starting angle of rocket, in degrees
    double time_s;
    double pressure_pa;
    double altitude_m;
    double temperature_k;
    double vertical_velocity_mps;
    double horizontal_velocity_mps;
};

/**
Converts one raw telemetry samle into the internal SI representation.

The conversion performs these operations:
- ms to s
- hPa to Pa
- C to K
- absolute altitude to relative altitude (relative to the launch reference altitude)

@param raw Raw telemetry sample from the flight log
@return Normalized state suitable for atmospheric, predictor, and simulator model.
*/
RocketState to_vertical_state(const RawTelemetrySample& raw);

/** 
derives horizontal velocity from vertical velocity and tilt for initialization

    @param RocketState current vertical flight state 

 */
void initialize_horizontal_velocity_from_tilt(RocketState &state);

}