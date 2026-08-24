#include "airbrake/apogee_predictor.hpp"
#include "airbrake/drag_table_csv.hpp"

#include <exception>
#include <iomanip>
#include <iostream>

const char* status_name(
    airbrake::PredictionStatus status
) {
    switch (status) {
        case airbrake::PredictionStatus::reached_apogee:
            return "reached apogee";

        case airbrake::PredictionStatus::timed_out:
            return "timed out";

        case airbrake::PredictionStatus::invalid_input:
            return "invalid input";
    }

    return "unknown";
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr
            << "Usage: airbrake_sim <drag-table.csv>\n";

        return 1;
    }

    try {
        const airbrake::DragTable drag_table =
            airbrake::load_drag_table_csv(argv[1]);

        const airbrake::SimulationConfig config{};

        const airbrake::ApogeePredictor predictor(
            config,
            drag_table
        );

        const airbrake::VerticalState initial_state{
            .time_s = 0.0,
            .pressure_pa = 101325.0,
            .altitude_m = 50.0,
            .temperature_k = 293.15,
            .vertical_velocity_mps = 120.0
        };

        std::cout
            << std::fixed
            << std::setprecision(3);

        for (int i = 0; i <= 10; ++i) {
            const double deployment_fraction =
                static_cast<double>(i) / 10.0;

            const airbrake::PredictionResult result =
                predictor.predict(
                    initial_state,
                    deployment_fraction
                );

            std::cout
                << "Deployment: "
                << deployment_fraction
                << ", Apogee: "
                << result.apogee_m
                << " m, Time: "
                << result.time_to_apogee_s
                << " s, Status: "
                << status_name(result.status)
                << '\n';
        }
    }
    catch (const std::exception& error) {
        std::cerr
            << "Error: "
            << error.what()
            << '\n';

        return 1;
    }

    return 0;
}