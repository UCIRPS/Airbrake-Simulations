#pragma once

namespace airbrake {

struct RawTelemetrySample {
    double time_ms;
    double pressure_hpa;
    double altitude_m;
    double start_altitude_m;
    double temperature_c;
    double vertical_velocity_mps;
};

struct VerticalState {
    double time_s;
    double pressure_pa;
    double altitude_m;
    double temperature_k;
    double vertical_velocity_mps;
};

VerticalState to_vertical_state(const RawTelemetrySample& raw);

}