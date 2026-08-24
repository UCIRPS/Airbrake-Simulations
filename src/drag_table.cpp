#include "airbrake/drag_table.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstddef>
#include <iterator>
#include <utility>

namespace airbrake {

namespace {

void validate_grid(const std::vector<double>& grid) {
    // The grid must contain at least two values for interpolation, be finite, and strictly increasing

    // Most have atleast tow values for interpolation
    if (grid.size() < 2) {
        throw std::invalid_argument(
            "Drag-table grids must contain at least two values"
        );
    }
    // The grid must contain finite values and be strictly increasing
    for (std::size_t i = 0; i < grid.size(); ++i) {
        // Check for finite values
        if (!std::isfinite(grid[i])) {
            throw std::invalid_argument(
                "Drag-table grid contains a non-finite value"
            );
        }
        // Strictly increasing check
        if (i > 0 && grid[i] <= grid[i - 1]) {
            throw std::invalid_argument(
                "Drag-table grids must be strictly increasing"
            );
        }
    }
}

std::size_t lower_index(
    // Returns the index of the lower grid value for interpolation
    const std::vector<double>& grid,
    double value
) {
    // find first value in grid that is greater than the input value
    const auto upper =
        std::upper_bound(grid.begin(), grid.end(), value);
    // if the upper bound is the first element, return 0
    if (upper == grid.begin()) {
        return 0;
    }
    // if the upper bound is the end of the grid, return the second to last index
    if (upper == grid.end()) {
        return grid.size() - 2;
    }
    // return the index of the lower value, which is one less than the upper bound
    return static_cast<std::size_t>(
        std::distance(grid.begin(), upper) - 1
    );
}
} // namespace

DragTable::DragTable(
    std::vector<double> deployment_grid,
    std::vector<double> mach_grid,
    std::vector<double> cda_values
)
  : deployment_grid_(std::move(deployment_grid)),
    mach_grid_(std::move(mach_grid)),
    cda_values_(std::move(cda_values)){

        validate_grid(deployment_grid_);
        validate_grid(mach_grid_);
        
        const std::size_t expected_values = deployment_grid_.size() * mach_grid_.size();

        if (cda_values_.size() != expected_values){
            throw std::invalid_argument(
                "Drag-table value count does not match grid dimensions"
            );
        }

        for (double value : cda_values_){
            if (!std::isfinite(value) || value < 0.0){
                throw std::invalid_argument(
                    "Drag-table values must be finite and non-negative"
                );
            }
        }
    }   

double DragTable::cda_m2(
    double deployment_fraction,
    double mach_number
) const {
    const double deployment = std::clamp(
        deployment_fraction,
        deployment_grid_.front(),
        deployment_grid_.back()
    );

    const double mach = std::clamp(
        mach_number,
        mach_grid_.front(),
        mach_grid_.back()
    );

    const std::size_t deployment_index = lower_index(deployment_grid_, deployment);
    
    const std::size_t mach_index = lower_index(mach_grid_, mach);

    const double deployment_fraction_local = (deployment - deployment_grid_[deployment_index]) / (deployment_grid_[deployment_index + 1] - deployment_grid_[deployment_index]);
    const double mach_fraction_local = (mach - mach_grid_[mach_index]) / (mach_grid_[mach_index + 1] - mach_grid_[mach_index]);

    const std::size_t mach_count = mach_grid_.size();

    const auto value_at = [&](std::size_t deployment_index_value, std::size_t mach_index_value){
        return cda_values_[deployment_index_value * mach_count + mach_index_value];
    };

    const double cda00 = value_at(deployment_index, mach_index);
    const double cda01 = value_at(deployment_index, mach_index + 1);
    const double cda10 = value_at(deployment_index + 1, mach_index);
    const double cda11 = value_at(deployment_index + 1, mach_index + 1);

    const double cda_at_lower_mach = cda00 + deployment_fraction_local * (cda10 - cda00);
    const double cda_at_upper_mach = cda01 + deployment_fraction_local * (cda11 - cda01);

    return cda_at_lower_mach + mach_fraction_local * (cda_at_upper_mach - cda_at_lower_mach);
}

} // namespace airbrake