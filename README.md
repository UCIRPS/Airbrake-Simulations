# Airbrake Simulations

`airbrake_sim` is a small C++20 vertical-flight simulator for choosing an airbrake deployment level from a predicted apogee. It combines a one-dimensional atmosphere and ascent dynamics model with a two-dimensional aerodynamic `CdA` lookup table. The executable starts from one hardcoded post-motor-burn state, advances in fixed time steps, and asks the deployment controller for a command at each step.

This is a model and controller exercise, not a complete flight-computer or telemetry-replay application. In particular, it has no thrust model, motor-burn phase, attitude/lateral dynamics, wind, actuator dynamics, or telemetry-file input pipeline.

## Get the source

```sh
git clone https://github.com/UCIRPS/Airbrake-Simulations.git
cd Airbrake-Simulations
```

## Prerequisites and build

You need:

- CMake 3.24 or newer
- A compiler with C++20 support (Clang or GCC)

From the repository root:

```sh
cmake -S . -B build
cmake --build build --parallel
```

The build produces `build/airbrake_sim`.

## Run

```text
./build/airbrake_sim <drag_table.csv> [target_apogee_m]
```

The first argument is required. The optional target is in metres and overrides the default target of 304.8 m. For example, using the checked-in table:

```sh
./build/airbrake_sim data/night_fury_cda.csv
./build/airbrake_sim data/night_fury_cda.csv 2580
```

An incorrect argument count prints a usage line and exits with status 1. File, CSV, and numeric-conversion failures are reported as `Error: ...` and also return status 1.

## Vehicle and simulation parameters

The shared defaults are in [`include/airbrake/simulation_config.hpp`](include/airbrake/simulation_config.hpp), inside `airbrake::SimulationConfig` (the field initializers are currently lines 10–19):

| Field | Default | Units / meaning |
| --- | ---: | --- |
| `mass_kg` | `30.39` | kg, vehicle mass |
| `target_apogee_m` | `304.8` | m, requested apogee; overridden by the CLI's second argument |
| `gravity_mps2` | `9.80665` | m/s², gravity |
| `gamma` | `1.4` | dimensionless, ratio of specific heats |
| `gas_constant_j_per_kg_k` | `287.05287` | J/(kg·K), specific gas constant for air |
| `temperature_lapse_rate_k_per_m` | `0.0065` | K/m, linear atmospheric lapse rate |
| `integration_dt_s` | `0.01` | s, physics step |
| `max_simulation_time_s` | `120.0` | s, predictor and main-loop time limit |

The initial post-burn state is a separate hardcoded block in [`src/main.cpp`](src/main.cpp) (currently lines 72–78):

```cpp
airbrake::VerticalState state{
    .time_s = 0.0,
    .pressure_pa = 83047.0,
    .altitude_m = 996.515,
    .temperature_k = 317.30,
    .vertical_velocity_mps = 200.27
};
```

The controller's constructor defaults are in [`include/airbrake/deployment_controller.hpp`](include/airbrake/deployment_controller.hpp) (currently lines 39–44): maximum deployment Mach `0.7` and `11` discrete deployment levels, which gives 10% increments from 0% through 100%. Change the source defaults or pass different values from a C++ caller; the current `main.cpp` uses the constructor defaults.

## Drag-table CSV

The loader requires this exact header and column order:

```text
deployment_fraction,mach,cda_m2
```

Each subsequent nonblank row is exactly three numeric, comma-separated values:

| Column | Units / meaning |
| --- | --- |
| `deployment_fraction` | dimensionless airbrake deployment fraction (normally 0.0 to 1.0) |
| `mach` | dimensionless Mach number |
| `cda_m2` | m², combined drag coefficient times reference area (`CdA`) |

Example rows from [`data/night_fury_cda.csv`](data/night_fury_cda.csv):

```text
deployment_fraction,mach,cda_m2
0.0,0.05,0.0089884
0.0,0.10,0.0085976
0.0,0.15,0.0084022
...
1.0,0.70,0.014228572
```

The file must describe a complete rectangular grid: every deployment value must appear once with every Mach value. The checked-in file has 11 deployment rows (`0.0` through `1.0` in 0.1 increments), 14 Mach columns (`0.05` through `0.70` in 0.05 increments), and 154 data records (155 logical records including the header). The loader rejects an unexpected header, malformed or extra fields, non-finite values, negative `CdA`, duplicate deployment/Mach pairs, missing grid pairs, and incomplete grids. Both axes must provide at least two finite, strictly increasing values after loading; rows may be supplied in a different order because the loader sorts them.

`DragTable::cda_m2()` performs bilinear interpolation between grid points. Deployment and Mach inputs outside the table are clamped to the nearest grid boundary before interpolation. The controller itself only requests fractions from 0.0 to 1.0 and inhibits deployment above Mach 0.7.

## Replacing the initial state with telemetry

The conversion API is already present, but `main.cpp` currently initializes `VerticalState` directly. To use one raw flight-computer sample, replace that initializer with code of this form:

```cpp
const airbrake::RawTelemetrySample raw{
    .time_ms = 1250.0,
    .pressure_hpa = 830.47,
    .altitude_m = 1096.515,
    .start_altitude_m = 100.0,
    .temperature_c = 44.15,
    .vertical_velocity_mps = 200.27
};
airbrake::VerticalState state = airbrake::to_vertical_state(raw);
```

Keep `state` mutable: `main.cpp` later assigns each `step_result.state` back into it as the simulation advances.

`RawTelemetrySample` is declared in [`include/airbrake/flight_sample.hpp`](include/airbrake/flight_sample.hpp). Its sensor units are:

- `time_ms`: milliseconds
- `pressure_hpa`: hectopascals (hPa)
- `altitude_m`: absolute altitude in metres
- `start_altitude_m`: launch-reference altitude in metres
- `temperature_c`: degrees Celsius
- `vertical_velocity_mps`: metres per second

`to_vertical_state()` converts milliseconds to seconds, hPa to Pa, Celsius to kelvin, and subtracts `start_altitude_m` from `altitude_m`. The resulting internal state uses seconds, pascals, relative metres, kelvin, and m/s. The example values above are illustrative input for the conversion API; they are not an additional simulator run.

There is currently no telemetry CSV parser, replay loop, or CLI argument for telemetry. The executable accepts only the drag-table path and optional target apogee, so replacing the initial state this way still handles one manually supplied sample. A replay pipeline would need to parse rows, convert each sample, and define how the controller/dynamics loop consumes them.

## Controller and output behavior

For each 0.01 s physics step while the vehicle is ascending, the controller:

1. Predicts the no-airbrake apogee.
2. Returns `prediction unavailable` with 0% if that baseline prediction does not reach apogee.
3. Returns `inactive` with 0% when velocity is non-positive or Mach is above 0.7.
4. Otherwise binary-searches the 11 commands from 0% to 100% and selects the highest level whose predicted apogee is at or above the configured target. If no nonzero level satisfies the target it selects 0%; if every level satisfies it selects 100%.

The console uses fixed-point values with three decimals. Status lines are printed at approximately 0.1 s intervals and immediately when deployment changes:

```text
Time: <s> s, Altitude: <m> m, Velocity: <m/s> m/s, Deployment: <percent>%, Predicted apogee: <m> m, Status: <status>
```

At the end it prints the maximum altitude reached and the final simulated time:

```text
Simulation complete
Apogee: <m> m
Time: <s> s
```

## Reproducible runs

The following excerpts were produced from the current executable after the build above, from the repository root, using `data/night_fury_cda.csv`. They include the first line, every deployment transition (where applicable), and the final summary.

### No deployment: target 100000 m

Command:

```sh
./build/airbrake_sim data/night_fury_cda.csv 100000
```

Output excerpt:

```text
Time: 0.000 s, Altitude: 996.515 m, Velocity: 200.270 m/s, Deployment: 0.000%, Predicted apogee: 2657.171 m, Status: commanded
Simulation complete
Apogee: 2657.171 m
Time: 17.819 s
```

The target is above the no-brake prediction, so no available command satisfies it and the selected deployment remains 0%.

### Full deployment: target 1 m

Command:

```sh
./build/airbrake_sim data/night_fury_cda.csv 1
```

Output excerpt:

```text
Time: 0.000 s, Altitude: 996.515 m, Velocity: 200.270 m/s, Deployment: 100.000%, Predicted apogee: 2510.577 m, Status: commanded
Simulation complete
Apogee: 2510.577 m
Time: 16.781 s
```

Even the fully deployed prediction is above the very low target, so the controller selects 100% immediately.

### Dynamic deployment: target 2580 m

Command:

```sh
./build/airbrake_sim data/night_fury_cda.csv 2580
```

Output excerpt:

```text
Time: 0.000 s, Altitude: 996.515 m, Velocity: 200.270 m/s, Deployment: 40.000%, Predicted apogee: 2594.377 m, Status: commanded
Time: 0.120 s, Altitude: 1020.431 m, Velocity: 198.338 m/s, Deployment: 50.000%, Predicted apogee: 2580.031 m, Status: commanded
Time: 13.430 s, Altitude: 2507.260 m, Velocity: 37.978 m/s, Deployment: 60.000%, Predicted apogee: 2580.000 m, Status: commanded
Time: 16.420 s, Altitude: 2576.419 m, Velocity: 8.383 m/s, Deployment: 70.000%, Predicted apogee: 2580.000 m, Status: commanded
Time: 16.890 s, Altitude: 2579.275 m, Velocity: 3.771 m/s, Deployment: 80.000%, Predicted apogee: 2580.000 m, Status: commanded
Time: 17.110 s, Altitude: 2579.867 m, Velocity: 1.613 m/s, Deployment: 90.000%, Predicted apogee: 2580.000 m, Status: commanded
Time: 17.170 s, Altitude: 2579.946 m, Velocity: 1.025 m/s, Deployment: 100.000%, Predicted apogee: 2580.000 m, Status: commanded
Simulation complete
Apogee: 2580.000 m
Time: 17.275 s
```

This run shows the controller recomputing as the state changes: it steps from 40% to 100% in 10% increments while the predicted apogee converges to the 2580 m target.

## Current limitations

- The model is one-dimensional and vertical; it has no thrust or motor-burn model, attitude, rotation, lateral motion, wind, or sensor uncertainty.
- Atmosphere is a simplified linear-lapse-rate/ideal-gas model. It is not a full atmospheric or weather model.
- `CdA` is a static table lookup. Deployment is instantaneous and quantized; actuator travel, rate limits, hysteresis, and position feedback are not modeled.
- The predictor runs a fresh fixed-step simulation for each candidate command and each controller update. There is no real-time scheduling or hardware interface.
- The simulator starts from one source-defined post-burn state and stops at apogee or the 120 s limit. It does not ingest or replay telemetry logs.
- No uncertainty analysis, calibration workflow, or automated comparison against flight data is included.
