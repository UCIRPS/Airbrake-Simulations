#include "airbrake/atmosphere.hpp"

#include <algorithm>
#include <cmath>

namespace airbrake {

Atmosphere::Atmosphere(const SimulationConfig& config)
    : config_(config) {}

double Atmosphere::temperature_at_delta_altitude(
    double reference_temperature_k,
    double delta_altitude_m
) const {
    const double temperature =
        reference_temperature_k
        - config_.temperature_lapse_rate_k_per_m * delta_altitude_m;

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

    const double exponent =
        config_.gravity_mps2
        / (
            config_.gas_constant_j_per_kg_k
            * config_.temperature_lapse_rate_k_per_m
        );

    return reference_pressure_pa
        * std::pow(local_temperature / reference_temperature_k, exponent);
}

double Atmosphere::density(
    double pressure_pa,
    double temperature_k
) const {
    return pressure_pa
        / (config_.gas_constant_j_per_kg_k * temperature_k);
}

double Atmosphere::speed_of_sound(
    double temperature_k
) const {
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
    return std::abs(velocity_mps)
        / speed_of_sound(temperature_k);
}

} 