#pragma once

#include "airbrake/vertical_dynamics.hpp"

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
    VerticalDynamics dynamics_;
};

} // namespace airbrake
