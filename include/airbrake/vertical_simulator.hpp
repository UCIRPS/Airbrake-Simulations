#pragma once

#include "airbrake/atmosphere.hpp"
#include "airbrake/drag_table.hpp"
#include "airbrake/flight_sample.hpp"
#include "airbrake/simulation_config.hpp"

namespace airbrake {

class VerticalSimulator {
public:
    VerticalSimulator(
        SimulationConfig config,
        DragTable drag_table
    );

    VerticalState step(
        const VerticalState& state,
        double deployment_fraction
    ) const;

private:
    SimulationConfig config_;
    Atmosphere atmosphere_;
    DragTable drag_table_;
};

} // namespace airbrake