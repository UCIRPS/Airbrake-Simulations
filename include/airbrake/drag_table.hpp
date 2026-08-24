#pragma once

#include <vector>

namespace airbrake {

class DragTable {
public:
    DragTable(
        std::vector<double> deployment_grid, // 0.0 to 1.0 deployment fraction
        std::vector<double> mach_grid,  // 0.05 to 0.7 mach number
        std::vector<double> cda_values // coefficient of drag multiplied by frontal area
    );

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