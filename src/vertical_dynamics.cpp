#include "airbrake/vertical_dynamics.hpp"

#include <cmath>
#include <utility>

namespace airbrake {

namespace {

// Checks whether the physical configuration contains usable values.
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
        config.temperature_lapse_rate_k_per_m > 0.0;
}

// Checks whether the current flight state is physically usable.
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

VerticalDynamics::VerticalDynamics(
    SimulationConfig config,
    DragTable drag_table
)
    : config_(std::move(config)),
      atmosphere_(config_),
      drag_table_(std::move(drag_table)) {
}

// Advances the vertical flight model by one step
VerticalStepResult VerticalDynamics::step(
    const VerticalState& state,
    double deployment_fraction,
    double dt_s
) const {
    // Reject invalid input before performing any physics calculations
    if (
        !valid_config(config_) ||
        !valid_state(state) ||
        !std::isfinite(deployment_fraction) ||
        !std::isfinite(dt_s) ||
        dt_s <= 0.0
    ) {
        return VerticalStepResult{
            .state = state,
            .status = VerticalStepStatus::invalid_input
        };
    }

    // If the vertical velocity is already zero, the vehicle is at apogee.
    if (state.vertical_velocity_mps == 0.0) {
        return VerticalStepResult{
            .state = state,
            .status = VerticalStepStatus::reached_apogee
        };
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

    // Calculate aerodynamic drag:
    /// F_drag = 0.5 * density * velocity^2 * CdA
    const double drag_force_n =
        0.5 *
        density_kg_per_m3 *
        state.vertical_velocity_mps *
        state.vertical_velocity_mps *
        cda_m2;

    // Calculate vertical acceleration
    // Gravity acts downward, and drag also opposes the upward motion.
    // Therefore both terms are negative while the vehicle is ascending.
    const double acceleration_mps2 =
        -config_.gravity_mps2 -
        drag_force_n / config_.mass_kg;

    // Estimate the velocity at the end of the full time step using
    // constant acceleration over this step.
    const double predicted_velocity_mps =
        state.vertical_velocity_mps +
        acceleration_mps2 * dt_s;

    double actual_dt_s = dt_s;
    double next_velocity_mps = predicted_velocity_mps;
    VerticalStepStatus status = VerticalStepStatus::advanced;

    // Non-positive predicted velocity means the vehicle reaches apogee somewhere inside this time step
    if (predicted_velocity_mps <= 0.0) {
        // Estimate fraction of the time step needed for velocity to decrease from it's current value to zero
        const double crossing_fraction =
            state.vertical_velocity_mps /
            (state.vertical_velocity_mps - predicted_velocity_mps);
        // Shorten the step
        actual_dt_s = dt_s * crossing_fraction;
        // Reached apogee
        next_velocity_mps = 0.0;
        status = VerticalStepStatus::reached_apogee;
    }
    
    // Calculate altitude change using constat acceleration equation 
    const double altitude_change_m =
        state.vertical_velocity_mps * actual_dt_s +
        0.5 *
        acceleration_mps2 *
        actual_dt_s *
        actual_dt_s;

    // Copy current state and advance it one time step
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

    return VerticalStepResult{
        .state = next_state,
        .status = status
    };
}

} // namespace airbrake
