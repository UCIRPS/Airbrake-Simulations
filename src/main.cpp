#include "airbrake/deployment_controller.hpp"
#include "airbrake/drag_table_csv.hpp"
#include "airbrake/vertical_dynamics.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace {

// Converts an internal controller status into readable console text.
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
    // Require the drag-table path and allow an optional target apogee.
    //
    // Expected usage:
    //   ./airbrake_sim <drag_table.csv> [target_apogee_m]
    if (argc < 2 || argc > 3) {
        std::cerr
            << "Usage: " << argv[0]
            << " <drag_table.csv> [target_apogee_m]\n";
        return 1;
    }

    // Catch file, parsing, conversion, and simulation errors in one place.
    try {
        // Start with the default vehicle and simulation configuration.
        airbrake::SimulationConfig config;
        
        // If a target apogee was provided, replace the default target.
        if (argc == 3) {
            config.target_apogee_m = std::stod(argv[2]);
        }

        // Load and validate the aerodynamic drag table from the CSV file.
        const airbrake::DragTable drag_table =
            airbrake::load_drag_table_csv(argv[1]);

        // The controller predicts apogee and selects airbrake deployment.
        airbrake::DeploymentController controller(
            config,
            drag_table
        );

        // The dynamics model advances the vehicle by one physics step.
        airbrake::VerticalDynamics dynamics(
            config,
            drag_table
        );

        // Define the initial vertical flight state.
        airbrake::VerticalState state{
            .time_s = 0.0,
            .pressure_pa = 83047.0,
            .altitude_m = 996.515,
            .temperature_k = 317.30,
            .vertical_velocity_mps = 200.27
        };

        // Used to limit regular console output to approximately every 0.1 seconds of simulated time.
        double next_print_time_s = 0.0;
        // Track previous deployment
        double previous_deployment = -1.0;

        // Print decimal values in fixed-point format with three decimals.
        std::cout << std::fixed << std::setprecision(3);

        // Track the highest altitude reached during the simulation.
        double maximum_altitude_m = state.altitude_m;

        // Continue while the vehicle is ascending and the simulation has
        // not exceeded its configured time limit.
        while (
            state.vertical_velocity_mps > 0.0 &&
            state.time_s < config.max_simulation_time_s
        ) {
            // Use the current state to calculate the airbrake command.
            const airbrake::DeploymentCommand command =
                controller.compute(state);
            
                // Detect whether the controller selected a new deployment level.
            const bool deployment_changed =
                command.deployment_fraction != previous_deployment;

            // Print periodically or immediately when deployment changes.
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

                // Save the current deployment so the next iteration can  detect whether it changed.
                previous_deployment =
                    command.deployment_fraction;
                
                    // Schedule the next regular status message.
                next_print_time_s =
                    state.time_s + 0.100;
            }

            // Advance the physics model by one configured integration step.
            const airbrake::VerticalStepResult step_result =
                dynamics.step(
                    state,
                    command.deployment_fraction,
                    config.integration_dt_s
                );

            // Stop with an exception if the physics model rejects the input.
            if (
                step_result.status ==
                airbrake::VerticalStepStatus::invalid_input
            ) {
                throw std::invalid_argument(
                    "Invalid input to VerticalDynamics::step"
                );
            }

            // Replace the current state with the updated state returned by the physics model.
            state = step_result.state;

            // Preserve the greatest altitude reached so far.
            maximum_altitude_m = std::max(
                maximum_altitude_m,
                state.altitude_m
            );
        }
        
        // Print the final simulation summary.
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
