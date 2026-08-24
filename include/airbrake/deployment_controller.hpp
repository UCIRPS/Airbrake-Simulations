#pragma once

#include <cstddef>

#include "airbrake/apogee_predictor.hpp"
#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/flight_sample.hpp"
#include "airbrake/simulation_config.hpp"

namespace airbrake {

enum class ControllerStatus {
    commanded,
    inactive, 
    prediction_unavailable,
    invalid_input
};

struct DeploymentCommand {
    double deployment_fraction;
    PredictionResult prediction;
    ControllerStatus status;
};

class DeploymentController {
public:
    DeploymentController(
        SimulationConfig config,
        DragTable drag_table,
        double max_mach = 0.7,
        std::size_t deployment_levels = 11
    );
    DeploymentCommand compute(
        const VerticalState& state
    ) const;

private:
    SimulationConfig config_;
    Atmosphere atmosphere_;
    ApogeePredictor predictor_;
    double max_mach_;
    std::size_t deployment_levels_;
};

}