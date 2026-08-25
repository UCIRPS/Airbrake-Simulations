#include "airbrake/deployment_controller.hpp"
#include "airbrake/drag_table_csv.hpp"
#include "airbrake/vertical_simulator.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <algorithm>

namespace {

const char* controller_status_to_string(
    airbrake::ControllerStatus status
) {
    switch (status) {
        case airbrake::ControllerStatus::commanded:
            return "commanded";
        case airbrake::ControllerStatus::inactive:
            return "inactive";
        case airbrake::ControllerStatus::prediction_unavailable:
            return "prediction unavailable";
        case airbrake::ControllerStatus::invalid_input:
            return "invalid input";
    }

    return "unknown";
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr
            << "Usage: " << argv[0]
            << " <drag_table.csv> [target_apogee_m]\n";
        return 1;
    }

    try {
        airbrake::SimulationConfig config;

        if (argc == 3) {
            config.target_apogee_m = std::stod(argv[2]);
        }

        const airbrake::DragTable drag_table =
            airbrake::load_drag_table_csv(argv[1]);

        airbrake::DeploymentController controller(
            config,
            drag_table
        );

        airbrake::VerticalSimulator simulator(
            config,
            drag_table
        );

        airbrake::VerticalState state{
            .time_s = 0.0,
            .pressure_pa = 83047.0,
            .altitude_m = 996.515,
            .temperature_k = 317.30,
            .vertical_velocity_mps = 200.27
        };

        double next_print_time_s = 0.0;
        double previous_deployment = -1.0;

        std::cout << std::fixed << std::setprecision(3);

        double maximum_altitude_m = state.altitude_m;

        while (
            state.vertical_velocity_mps > 0.0 &&
            state.time_s < config.max_simulation_time_s
        ) {
            const airbrake::DeploymentCommand command =
                controller.compute(state);

            const bool deployment_changed =
                command.deployment_fraction != previous_deployment;

            if (
                state.time_s >= next_print_time_s ||
                deployment_changed
            ) {
                std::cout
                    << "Time: " << state.time_s
                    << " s, Altitude: " << state.altitude_m
                    << " m, Velocity: "
                    << state.vertical_velocity_mps
                    << " m/s, Deployment: "
                    << command.deployment_fraction * 100.0
                    << "%, Predicted apogee: "
                    << command.prediction.apogee_m
                    << " m, Status: "
                    << controller_status_to_string(command.status)
                    << '\n';

                previous_deployment =
                    command.deployment_fraction;

                next_print_time_s =
                    state.time_s + 0.100;
            }

            state = simulator.step(
                state,
                command.deployment_fraction
            );
            maximum_altitude_m = std::max(
                maximum_altitude_m,
                state.altitude_m
            );
        }

        std::cout << "\nSimulation complete\n";
        std::cout << "Apogee: "
                  << maximum_altitude_m
                  << " m\n";
        std::cout << "Time: "
                  << state.time_s
                  << " s\n";
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}