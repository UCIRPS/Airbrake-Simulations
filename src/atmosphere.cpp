#include "airbrake/atmosphere.hpp"

#include <algorithm>
#include <cmath>

namespace airbrake {

Atmosphere::Atmosphere(const SimulationConfig& config)
    : config_(config) {}
    // The atmosphere keeps a reference to the caller's configuration

double Atmosphere::temperature_at_delta_altitude(
    double reference_temperature_k,
    double delta_altitude_m
) const {
    // A lapse-rate model assumes temperature decreases linearly as altitude increase.
    // A negative altitude change increases temperature
    const double temperature =
        reference_temperature_k
        - config_.temperature_lapse_rate_k_per_m * delta_altitude_m;

    // Prevent simplified model from producing zero or negative temperature
    return std::max(temperature, 1.0);
}


double Atmosphere::pressure_at_delta_altitude(
    double reference_pressure_pa,
    double reference_temperature_k,
    double delta_altitude_m
) const {
    const double local_temperature =
        temperature_at_delta_altitude(
            reference_temperature_k,
            delta_altitude_m
        );
    
    // This is the exponent from the barometric formula for an atmosphere with a linear
    // temperature lapse rate
    const double exponent =
        config_.gravity_mps2
        / (
            config_.gas_constant_j_per_kg_k
            * config_.temperature_lapse_rate_k_per_m
        );
    // Pressure decreases as the rocket climbs and increases when it moves below the reference altitude
    return reference_pressure_pa
        * std::pow(local_temperature / reference_temperature_k, exponent);
}

double Atmosphere::density(
    double pressure_pa,
    double temperature_k
) const {
    // Ideal gas law rearranged to solve for air density
    // rho = p / (R * T)
    return pressure_pa
        / (config_.gas_constant_j_per_kg_k * temperature_k);
}

double Atmosphere::speed_of_sound(
    double temperature_k
) const {
    // Speed of sound for an ideal gas
    // a = sqrt(gamma * R * T)
    return std::sqrt(
        config_.gamma
        * config_.gas_constant_j_per_kg_k
        * temperature_k
    );
}

double Atmosphere::mach(
    double velocity_mps,
    double temperature_k
) const {
    // Mach number is the ratio of speed to the local speed of sound.
    // Use magnitude because Mach number is not directional.
    return std::abs(velocity_mps)
        / speed_of_sound(temperature_k);
}

} // namespace airbrake
