#include "airbrake/flight_sample.hpp"

namespace airbrake {

VerticalState to_vertical_state(const RawTelemetrySample& raw){
    VerticalState state{};

    state.time_s = raw.time_ms / 1000.0;
    state.pressure_pa = raw.pressure_hpa * 100.0;
    state.altitude_m = raw.altitude_m - raw.start_altitude_m;
    state.temperature_k = raw.temperature_c + 273.15;
    state.vertical_velocity_mps = raw.vertical_velocity_mps;

    return state;
}

}