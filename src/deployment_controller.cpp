#include "airbrake/deployment_controller.hpp"
#include "airbrake/apogee_predictor.hpp"
#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/simulation_config.hpp"

#include <cmath>
#include <utility>

namespace airbrake {
namespace {

// Validated the flight state before giving it to the controller
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

// Creates a consistent response for invalid controller input.
DeploymentCommand invalid_command(){
    return DeploymentCommand{
        .deployment_fraction = 0.0,
        .prediction = PredictionResult{
            .apogee_m = 0.0, 
            .time_to_apogee_s = 0.0, 
            .status = PredictionStatus::invalid_input
        },
        .status = ControllerStatus::invalid_input
    };
}

} // namespace

DeploymentController::DeploymentController(
    SimulationConfig config,
    DragTable drag_table,
    double max_mach, 
    std::size_t deployment_levels
) : 
    config_(std::move(config)),
    atmosphere_(config_),
    predictor_(config_, std::move(drag_table)),
    max_mach_(max_mach),
    deployment_levels_(deployment_levels){}

// Calculates the deployment command for the current flight state
DeploymentCommand DeploymentController::compute(
    const VerticalState& state
) const {
    // Reject invalid state or controller configuration before making any prediction.
    if (!valid_state(state) ||
        !std::isfinite(config_.target_apogee_m) ||
        config_.target_apogee_m <= 0.0 ||
        !std::isfinite(max_mach_) ||
        max_mach_ < 0.0 ||
        deployment_levels_ < 2){
            return invalid_command();
        }

    // First predict the apogee with no airbrake deployment.
    // This provides the baseline prediction and verifies that the
    // predictor can successfully simulate the current state.
    const PredictionResult no_brake_prediction = predictor_.predict(state, 0.0);

    if (no_brake_prediction.status != PredictionStatus::reached_apogee){
        return DeploymentCommand{
            .deployment_fraction = 0.0,
            .prediction = no_brake_prediction,
            .status = ControllerStatus::prediction_unavailable
        };
    }

    // Calculate the current Mach number for deployment safety checks.
    const double mach_number = atmosphere_.mach(state.vertical_velocity_mps, state.temperature_k);

    // Keep the airbrakes retracted after apogee or while the vehicle is
    // traveling faster than the allowed Mach limit.
    if (state.vertical_velocity_mps <= 0.0 || mach_number > max_mach_){
        return DeploymentCommand{
            .deployment_fraction = 0.0, 
            .prediction = no_brake_prediction,
            .status = ControllerStatus::inactive
        };
    }

    // Deployment levels are represented by integer indicies
    // index 0 -> 0.0 deployment
    // index 5 -> 0.5 deployment
    // index 10 -> 1.0 deployment
    std::size_t low = 0;
    std::size_t high = deployment_levels_ - 1;

    PredictionResult best_prediction = no_brake_prediction;

    // Search for the highest deployment level that still predicts an apogee at or above the target.
    while (low < high){
        // Choose the upper middle index
        const std::size_t middle = low + (high - low + 1) / 2;

        // Convert discrete index into normalized deployment fraction.
        const double deployment_fraction = 
            static_cast<double> (middle) / static_cast<double> (deployment_levels_ - 1);

        // Predict the apogee using this candidate deployment level.
        const PredictionResult prediction = predictor_.predict(state, deployment_fraction);

        if (prediction.status != PredictionStatus::reached_apogee){
            return DeploymentCommand{
                .deployment_fraction = 0.0,
                .prediction = prediction,
                .status = ControllerStatus::prediction_unavailable
            };
        }

        if (prediction.apogee_m >= config_.target_apogee_m){
            low = middle;
            best_prediction = prediction;
        } else {
            high = middle - 1;
        }
    }
    // If no deployment level above zero was acceptable, use the original no-brake prediction.
    if (low == 0){
        best_prediction = no_brake_prediction;
    }

    const double deployment_fraction = static_cast<double>(low) / static_cast<double>(deployment_levels_ - 1);

    return DeploymentCommand{
        .deployment_fraction = deployment_fraction,
        .prediction = best_prediction, 
        .status = ControllerStatus::commanded
    };
}

} // namespace airbrake