#pragma once

#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/flight_sample.hpp"
#include "airbrake/simulation_config.hpp"

namespace airbrake {

// Describes the result of advancing the vertical flight model by one step
enum class VerticalStepStatus {
    advanced,
    reached_apogee,
    invalid_input
};

// Contains the updated state and result status from one physics step
struct VerticalStepResult {
    VerticalState state;
    VerticalStepStatus status;
};

// Performs one step of the vehicle's vertical flight physics
class VerticalDynamics {
public:
    /**
        Creates a dynamic model with the provided simulation configuration 
        and aerodynamic drag table
    */
    VerticalDynamics(
        SimulationConfig config,
        DragTable drag_table
    );

    /**
        Advances the flight state by one time step

        @param state Current vertical flight state
        @param deployment_fraction Airbrake deployment from 0.0 to 1.0
        @param dt_s Duration of the physics model in seconds
        @return VerticalStepResult (updated state and step status)
    */

    VerticalStepResult step(
        const VerticalState& state,
        double deployment_fraction,
        double dt_s
    ) const;

private:
    // Physical constants and integration settings
    SimulationConfig config_;
    // Calculates the atmospheric properties during the step
    Atmosphere atmosphere_;
    // Provides CdA values of the current deployment and Mach number
    DragTable drag_table_;
};

} // namespace airbrake
