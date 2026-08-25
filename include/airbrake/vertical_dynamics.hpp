#pragma once

#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/flight_sample.hpp"
#include "airbrake/simulation_config.hpp"

namespace airbrake {

enum class VerticalStepStatus {
    advanced,
    reached_apogee,
    invalid_input
};

struct VerticalStepResult {
    VerticalState state;
    VerticalStepStatus status;
};

class VerticalDynamics {
public:
    VerticalDynamics(
        SimulationConfig config,
        DragTable drag_table
    );

    VerticalStepResult step(
        const VerticalState& state,
        double deployment_fraction,
        double dt_s
    ) const;

private:
    SimulationConfig config_;
    Atmosphere atmosphere_;
    DragTable drag_table_;
};

} // namespace airbrake
