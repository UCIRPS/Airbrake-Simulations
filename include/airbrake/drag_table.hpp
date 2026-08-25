#pragma once

#include <vector>

namespace airbrake {

/**
    Stores and interpolates a two-dimensional drag-area table. 

    The table uses deployment fraction and Mach numbers as input.
    It's output is the combined drag coefficient and reference area, CdA, in square meters. 
*/
class DragTable {
public:
    /**
        Creates a drag table

        @param deployment_grid Sorted deployment fractions, normally 0.0 to 1.0.
        @param mach_grid Sorted Mach-number sample points.
        @param cda_values Flattened row-major CdA values.

        The number of values must equal:
        deployment_grid.size() * mach_grid.size()

        Each deployment row contains one CdA value for every Mach Value
    */
    DragTable(
        std::vector<double> deployment_grid, // 0.0 to 1.0 deployment fraction
        std::vector<double> mach_grid,  // 0.05 to 0.7 mach number
        std::vector<double> cda_values // coefficient of drag multiplied by frontal area
    );

    /**
        Looks up CdA for a deployment and Mach number.
    
        Values between grid points are interpolated. Inputs outside
        the grid are clamped to the nearest supported grid boundary.
    
        @param deployment_fraction Airbrake deployment from 0.0 to 1.0.
        @param mach Mach number, dimensionless.
        @return Combined drag coefficient and area, in m^2.
    */

    double cda_m2( 
        double deployment_fraction,
        double mach
    ) const;

private:
    std::vector<double> deployment_grid_;
    std::vector<double> mach_grid_;
    std::vector<double> cda_values_;
};

}