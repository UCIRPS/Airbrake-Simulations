#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "airbrake/flight_sample.hpp"

TEST_CASE("converts raw telemetry into SI vertical state") {
    const airbrake::RawTelemetrySample raw{
        .time_ms = 1250.0,
        .pressure_hpa = 1013.25,
        .altitude_m = 150.0,
        .start_altitude_m = 100.0,
        .temperature_c = 20.0,
        .vertical_velocity_mps = 120.0
    };

    const airbrake::VerticalState state =
        airbrake::to_vertical_state(raw);

    REQUIRE(state.time_s == Catch::Approx(1.25));
    REQUIRE(state.pressure_pa == Catch::Approx(101325.0));
    REQUIRE(state.altitude_m == Catch::Approx(50.0));
    REQUIRE(state.temperature_k == Catch::Approx(293.15));
    REQUIRE(state.vertical_velocity_mps == Catch::Approx(120.0));
}