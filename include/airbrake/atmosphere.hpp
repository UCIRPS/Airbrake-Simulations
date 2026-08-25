#pragma once

#include "airbrake/simulation_config.hpp"

namespace airbrake {

/**
    Provide atmospheric properties for the vertical flight model.

    The model uses a simplified troposphere with a linear temperature lap[se rate and the ideal gas law.
*/
class Atmosphere {
public:
    /**
        Creates an atmospheric model using the supplied configuration.

        The Atmosphere object does not own the configuration. The referenced SimulationConfig 
        must remain alive for the entire lifetime of this object
    */
    explicit Atmosphere(const SimulationConfig& config);

    /**
        Calculates temperature after moving vertically from a reference point.

        @param reference_temperature_k  Temperature at the reference point, K.
        @param delta_altitude_m Change in altitude from the reference point, m.
                                Positive values mean upward motion.
        @return Local atmospheric temperature, K. 

        The result is limited to a minimum of 1 K.
    */
    double temperature_at_delta_altitude(
        // Calculate temperature after moving a certain distance upward or downward 
        // from a reference altitude
        double reference_temperature_k,
        double delta_altitude_m
    ) const;
    
    /**
        Calculates pressure after moving vertically from a reference point.
     
        @param reference_pressure_pa Pressure at the reference point, Pa.
        @param reference_temperature_k Temperature at the reference point, K.
        @param delta_altitude_m Change in altitude from the reference point, m.
                                Positive values mean upward motion.
        @return Local atmospheric pressure, Pa.
     
        This uses the barometric pressure relationship for a region with a
        linear temperature lapse rate.
     */
    double pressure_at_delta_altitude(
        // Calculates pressure after moving relative to a known pressure and temperature.
        double reference_pressure_pa,
        double reference_temperature_k,
        double delta_altitude_m
    ) const;

     /**
        Calculates air density using the ideal gas law.
     
        @param pressure_pa Local atmospheric pressure, Pa.
        @param temperature_k Local atmospheric temperature, K.
        @return Air density, kg/m^3.
     */
    double density(
        double pressure_pa,
        double temperature_k
    ) const;
    /**
        Calculates the local speed of sound.
    
        @param temperature_k Local atmospheric temperature, K.
        @return Speed of sound, m/s.
     */
    double speed_of_sound(
        double temperature_k
    ) const;
    
    /**
        Calculates Mach number from velocity and local temperature.

        The velocity magnitude is used, so upward and downward velocities
        produce a non-negative Mach number.

        @param velocity_mps Velocity, m/s.
        @param temperature_k Local atmospheric temperature, K.
        @return Mach number.
     */
    double mach(
        double velocity_mps,
        double temperature_k
    ) const;

private:
    // Non-owning reference to the configuration used by the atmospheric model. 
    const SimulationConfig& config_;
};

} //namespace airbrake