#pragma once

namespace airbrake {

struct SimulationConfig {
    double mass_kg = 30.39;
    double target_apogee_m = 304.8;

    double gravity_mps2 = 9.80665;
    double gamma = 1.4;
    double gas_constant_j_per_kg_k = 287.05287;
    double temperature_lapse_rate_k_per_m = 0.0065;

    double integration_dt_s = 0.01;
    double max_simulation_time_s = 120.0;
};

}