#pragma once

#include "airbrake/simulation_config.hpp"

namespace airbrake {

class Atmosphere {
public:
    explicit Atmosphere(const SimulationConfig& config);

    double temperature_at_delta_altitude(
        // Calculate temperature after moving a certain distance upward or downward 
        // from a reference altitude
        double reference_temperature_k,
        double delta_altitude_m
    ) const;
    
    double pressure_at_delta_altitude(
        // Calculates pressure after moving relative to a known pressure and temperature.
        double reference_pressure_pa,
        double reference_temperature_k,
        double delta_altitude_m
    ) const;

    double density(
        double pressure_pa,
        double temperature_k
    ) const;

    double speed_of_sound(
        double temperature_k
    ) const;

    double mach(
        double velocity_mps,
        double temperature_k
    ) const;

private:
    const SimulationConfig& config_;
};

}