#include "airbrake/vertical_simulator.hpp"

#include <stdexcept>
#include <utility>

namespace airbrake {

VerticalSimulator::VerticalSimulator(
    SimulationConfig config,
    DragTable drag_table
)
    : config_(std::move(config)),
      dynamics_(config_, std::move(drag_table)) {
}

VerticalState VerticalSimulator::step(
    const VerticalState& state,
    double deployment_fraction
) const {
    const VerticalStepResult result =
        dynamics_.step(
            state,
            deployment_fraction,
            config_.integration_dt_s
        );

    if (result.status == VerticalStepStatus::invalid_input) {
        throw std::invalid_argument(
            "Invalid input to VerticalSimulator::step"
        );
    }

    return result.state;
}

} // namespace airbrake
