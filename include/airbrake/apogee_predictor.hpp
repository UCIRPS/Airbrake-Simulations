#pragma once

#include "airbrake/vertical_dynamics.hpp"
#include "airbrake/flight_sample.hpp"
#include "airbrake/simulation_config.hpp"

namespace airbrake {

// Describes how the apogee prediction finished
enum class PredictionStatus {
    reached_apogee, 
    timed_out, 
    invalid_input
};

// Contains the result of an apogee prediction
struct PredictionResult {
    double apogee_m;
    double time_to_apogee_s;
    PredictionStatus status;
};

// Simulates the vehicle's remaining upward flight to estimate apogee
class ApogeePredictor {
public:
    // Creates a predictor with a simulation configuration and drag table
    ApogeePredictor(
        SimulationConfig config,
        DragTable drag_table
    );

    /**
        Predicts apogee using a fixed airbrake deployment fraction

        @param initial_state Current normalized flight state
        @param deployment_fraction Airbrake deployment from 0.0 to 1.0
        @return PredictionResult (Predicted apogee, time-to-apogee, and completion status)

    */
    PredictionResult predict(
        const VerticalState& initial_state,
        double deployment_fraction
    ) const;

private:
    // Simulation constants and physical parameters
    SimulationConfig config_;
    // Calculates the atmospheric properties during prediction
    VerticalDynamics dynamics_;
};

} // namespace airbrake
