#include "airbrake/apogee_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace airbrake {

namespace {

// Checks whether all simulation configuration values are usable
bool valid_config(const SimulationConfig& config) {
    return
        std::isfinite(config.mass_kg)
        && config.mass_kg > 0.0
        && std::isfinite(config.gravity_mps2)
        && config.gravity_mps2 > 0.0
        && std::isfinite(config.gamma)
        && config.gamma > 0.0
        && std::isfinite(config.gas_constant_j_per_kg_k)
        && config.gas_constant_j_per_kg_k > 0.0
        && std::isfinite(
            config.temperature_lapse_rate_k_per_m
        )
        && config.temperature_lapse_rate_k_per_m > 0.0
        && std::isfinite(config.integration_dt_s)
        && config.integration_dt_s > 0.0
        && std::isfinite(config.max_simulation_time_s)
        && config.max_simulation_time_s > 0.0;
}

// Checks whether the incoming flight state contains valid values.
bool valid_state(const VerticalState& state) {
    return
        std::isfinite(state.time_s)
        && std::isfinite(state.pressure_pa)
        && state.pressure_pa > 0.0
        && std::isfinite(state.altitude_m)
        && std::isfinite(state.temperature_k)
        && state.temperature_k > 0.0
        && std::isfinite(state.vertical_velocity_mps)
        && state.vertical_velocity_mps >= 0.0;
}
} // namespace

ApogeePredictor::ApogeePredictor(
    SimulationConfig config,
    DragTable drag_table
) : config_(std::move(config)),
    dynamics_(config_, std::move(drag_table)){}

PredictionResult ApogeePredictor::predict(
    const VerticalState& initial_state,
    double deployment_fraction
) const {
    // Reject invalid configuration, state, or deployment input before beginning simulation
    if (
        !valid_config(config_) || 
        !valid_state(initial_state) || 
        !std::isfinite(deployment_fraction)
    ) {
        return PredictionResult{
            .apogee_m = 0.0,
            .time_to_apogee_s = 0.0,
            .status = PredictionStatus::invalid_input
        };
    }

    // If the vehicle is already stationary vehicle, it is considered to be at apogee
    if (initial_state.vertical_velocity_mps == 0.0){
        return PredictionResult{
            .apogee_m = initial_state.altitude_m,
            .time_to_apogee_s = 0.0,
            .status = PredictionStatus::reached_apogee
        };
    }
    
    VerticalState state = initial_state;
    double elapsed_time_s = 0.0; 

    // Continue simulation until apogee is reached or the time limit expires
    while (
        elapsed_time_s < config_.max_simulation_time_s
    ) {
        const double remaining_time_s = config_.max_simulation_time_s - elapsed_time_s;
        const double dt_s = std::min(config_.integration_dt_s, remaining_time_s);

        // Advance the shared vertical physics model by only one time step.
        const VerticalStepResult step_result =
            dynamics_.step(
                state,
                deployment_fraction,
                dt_s
            );

        if (step_result.status == VerticalStepStatus::invalid_input) {
            return PredictionResult{
                .apogee_m = 0.0,
                .time_to_apogee_s = 0.0,
                .status = PredictionStatus::invalid_input
            };
        }

        state = step_result.state; // Store the state produced by the physics step
        elapsed_time_s = state.time_s - initial_state.time_s; 

        // If the dynamics model detected the upward velocity crossing zero, return the interpolated apogee state
        if (step_result.status == VerticalStepStatus::reached_apogee) {
            return PredictionResult{
                .apogee_m = state.altitude_m,
                .time_to_apogee_s = elapsed_time_s,
                .status = PredictionStatus::reached_apogee
            };
        }
    }

    return PredictionResult{
        .apogee_m = state.altitude_m,
        .time_to_apogee_s = elapsed_time_s,
        .status = PredictionStatus::timed_out
    };
}


} // namespace airbrake
