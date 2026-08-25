#include "airbrake/vertical_simulator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace airbrake {

namespace {

bool valid_config(const SimulationConfig& config) {
    return
        std::isfinite(config.mass_kg) &&
        config.mass_kg > 0.0 &&
        std::isfinite(config.gravity_mps2) &&
        config.gravity_mps2 > 0.0 &&
        std::isfinite(config.gamma) &&
        config.gamma > 0.0 &&
        std::isfinite(config.gas_constant_j_per_kg_k) &&
        config.gas_constant_j_per_kg_k > 0.0 &&
        std::isfinite(config.temperature_lapse_rate_k_per_m) &&
        config.temperature_lapse_rate_k_per_m > 0.0 &&
        std::isfinite(config.integration_dt_s) &&
        config.integration_dt_s > 0.0;
}

bool valid_state(const VerticalState& state) {
    return
        std::isfinite(state.time_s) &&
        std::isfinite(state.pressure_pa) &&
        state.pressure_pa > 0.0 &&
        std::isfinite(state.altitude_m) &&
        std::isfinite(state.temperature_k) &&
        state.temperature_k > 0.0 &&
        std::isfinite(state.vertical_velocity_mps) &&
        state.vertical_velocity_mps >= 0.0;
}

} // namespace

VerticalSimulator::VerticalSimulator(
    SimulationConfig config,
    DragTable drag_table
)
    : config_(std::move(config)),
      atmosphere_(config_),
      drag_table_(std::move(drag_table)) {
}

VerticalState VerticalSimulator::step(
    const VerticalState& state,
    double deployment_fraction
) const {
    if (
        !valid_config(config_) ||
        !valid_state(state) ||
        !std::isfinite(deployment_fraction)
    ) {
        throw std::invalid_argument(
            "Invalid input to VerticalSimulator::step"
        );
    }

    if (state.vertical_velocity_mps <= 0.0) {
        return state;
    }

    const double density_kg_per_m3 =
        atmosphere_.density(
            state.pressure_pa,
            state.temperature_k
        );

    const double mach_number =
        atmosphere_.mach(
            state.vertical_velocity_mps,
            state.temperature_k
        );

    const double cda_m2 =
        drag_table_.cda_m2(
            deployment_fraction,
            mach_number
        );

    const double drag_force_n =
        0.5 *
        density_kg_per_m3 *
        state.vertical_velocity_mps *
        state.vertical_velocity_mps *
        cda_m2;

    const double acceleration_mps2 =
        -config_.gravity_mps2 -
        drag_force_n / config_.mass_kg;

    const double dt_s = config_.integration_dt_s;

    const double predicted_velocity_mps =
        state.vertical_velocity_mps +
        acceleration_mps2 * dt_s;

    double actual_dt_s = dt_s;
    double next_velocity_mps = predicted_velocity_mps;

    if (predicted_velocity_mps <= 0.0) {
        actual_dt_s =
            state.vertical_velocity_mps /
            (state.vertical_velocity_mps - predicted_velocity_mps);

        next_velocity_mps = 0.0;
    }

    const double altitude_change_m =
        state.vertical_velocity_mps * actual_dt_s +
        0.5 *
        acceleration_mps2 *
        actual_dt_s *
        actual_dt_s;

    VerticalState next_state = state;

    next_state.time_s += actual_dt_s;
    next_state.altitude_m += altitude_change_m;
    next_state.vertical_velocity_mps = next_velocity_mps;

    next_state.temperature_k =
        atmosphere_.temperature_at_delta_altitude(
            state.temperature_k,
            altitude_change_m
        );

    next_state.pressure_pa =
        atmosphere_.pressure_at_delta_altitude(
            state.pressure_pa,
            state.temperature_k,
            altitude_change_m
        );

    return next_state;
}

} // namespace airbrake