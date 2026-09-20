#include "airbrake/flight_sample.hpp"

namespace airbrake {

RocketState to_vertical_state(const RawTelemetrySample& raw){
    RocketState state{};

    state.time_s = raw.time_ms / 1000.0;
    state.pressure_pa = raw.pressure_hpa * 100.0;
    state.altitude_m = raw.altitude_m - raw.start_altitude_m;
    state.temperature_k = raw.temperature_c + 273.15;
    state.vertical_velocity_mps = raw.vertical_velocity_mps;

    return state;
}

// derive horizontal velocity from vertical velocity and tilt
// use vy/tan() to calculate horizontal velocity
void initialize_horizontal_velocity_from_tilt(RocketState &state)
{
    const double tilt_rad = 
    state.tilt_from_vertical_deg * 
    (std::numbers::pi / 180.0);

    state.horizontal_velocity_mps =
        state.vertical_velocity_mps * 
        std::tan(tilt_rad);
    
}
} // namespace airbrake