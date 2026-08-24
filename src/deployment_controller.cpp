#include "airbrake/deployment_controller.hpp"
#include "airbrake/apogee_predictor.hpp"
#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/simulation_config.hpp"

#include <cmath>
#include <utility>

namespace airbrake {
namespace {

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

DeploymentCommand DeploymentController::compute(
    const VerticalState& state
) const {
    if (!valid_state(state) ||
        !std::isfinite(config_.target_apogee_m) ||
        config_.target_apogee_m <= 0.0 ||
        !std::isfinite(max_mach_) ||
        max_mach_ < 0.0 ||
        deployment_levels_ < 2){
            return invalid_command();
        }

    const PredictionResult no_brake_prediction = predictor_.predict(state, 0.0);

    if (no_brake_prediction.status != PredictionStatus::reached_apogee){
        return DeploymentCommand{
            .deployment_fraction = 0.0,
            .prediction = no_brake_prediction,
            .status = ControllerStatus::prediction_unavailable
        };
    }

    const double mach_number = atmosphere_.mach(state.vertical_velocity_mps, state.temperature_k);

    if (state.vertical_velocity_mps <= 0.0 || mach_number > max_mach_){
        return DeploymentCommand{
            .deployment_fraction = 0.0, 
            .prediction = no_brake_prediction,
            .status = ControllerStatus::inactive
        };
    }

    std::size_t low = 0;
    std::size_t high = deployment_levels_ - 1;

    PredictionResult best_prediction = no_brake_prediction;

    while (low < high){
        const std::size_t middle = low + (high - low + 1) / 2;

        const double deployment_fraction = 
            static_cast<double> (middle) / static_cast<double> (deployment_levels_ - 1);

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