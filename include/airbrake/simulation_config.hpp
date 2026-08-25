#pragma once

namespace airbrake {

/**
Configuration values shared by the atmosphere, dynamics, predictor,
simulator, and deployment controller.
 */
struct SimulationConfig {
    double mass_kg = 30.39; // vehicle mass
    double target_apogee_m = 304.8; // target apogee

    double gravity_mps2 = 9.80665; // gravitation acceleration
    double gamma = 1.4; // Ration of specific heats for air
    double gas_constant_j_per_kg_k = 287.05287; // Specific gas constant for air
    double temperature_lapse_rate_k_per_m = 0.0065; // Atmospheric temperature decreases with altitude

    double integration_dt_s = 0.01; // Physics integration timestep, in seconds
    double max_simulation_time_s = 120.0; // Maximum simulation time, in seconds
};

} // namespace airbrake /