#pragma once

#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/flight_sample.hpp"
#include "airbrake/simulation_config.hpp"

namespace airbrake {

enum class PredictionStatus {
    reached_apogee, 
    timed_out, 
    invalid_input
};

struct PredictionResult {
    double apogee_m;
    double time_to_apogee_s;
    PredictionStatus status;
};

class ApogeePredictor {
public:
    ApogeePredictor(
        SimulationConfig config,
        DragTable drag_table
    );

    PredictionResult predict(
        const VerticalState& initial_state,
        double deployment_fraction
    ) const;

private:
    SimulationConfig config_;
    Atmosphere atmosphere_;
    DragTable drag_table_;
};

}