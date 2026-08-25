#include "airbrake/drag_table.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstddef>
#include <iterator>
#include <utility>

namespace airbrake {

namespace {

/**
    Validates a grid used for interpolation.  

    Interpolation requires at least two points. The grid must also be finite
    and strictly increasing because lower_index() relies on binary search.
*/
void validate_grid(const std::vector<double>& grid) {
    if (grid.size() < 2) {
        throw std::invalid_argument(
            "Drag-table grids must contain at least two values"
        );
    }

    for (std::size_t i = 0; i < grid.size(); ++i) {

        if (!std::isfinite(grid[i])) {
            throw std::invalid_argument(
                "Drag-table grid contains a non-finite value"
            );
        }

        if (i > 0 && grid[i] <= grid[i - 1]) {
            throw std::invalid_argument(
                "Drag-table grids must be strictly increasing"
            );
        }
    }
}

/**
    Finds the lower grid index surrounding a value.
    
    The returned index is always valid for interpolation with index + 1.
    Values below or above the grid are assigned to the first or last
    interpolation interval, respectively.
 */

std::size_t lower_index(
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
        
        // Both axes must be valid before interpolation can be performed
        validate_grid(deployment_grid_);
        validate_grid(mach_grid_);
        
        // The table is rectangular: every deployment value needs one CdA value for every Mach value
        const std::size_t expected_values = deployment_grid_.size() * mach_grid_.size();

        if (cda_values_.size() != expected_values){
            throw std::invalid_argument(
                "Drag-table value count does not match grid dimensions"
            );
        }

        // A negative or non-finite CdA would make the drag model invalid.
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
    // Keep interpolation inside the supported table boundaries.
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

    // Convert the input values into local interpolation fractions between zero and one 
    // on their respective grid intervals. 0 = closest lowest. 1 = closest highest
    const double deployment_fraction_local = (deployment - deployment_grid_[deployment_index]) / (deployment_grid_[deployment_index + 1] - deployment_grid_[deployment_index]);
    const double mach_fraction_local = (mach - mach_grid_[mach_index]) / (mach_grid_[mach_index + 1] - mach_grid_[mach_index]);

    const std::size_t mach_count = mach_grid_.size();

    // The table is stored row-by-row
    // deployment row * number of Mach columns + Mach column

    const auto value_at = [&](std::size_t deployment_index_value, std::size_t mach_index_value){
        return cda_values_[deployment_index_value * mach_count + mach_index_value];
    };

    // Retrieve the four surrounding values for bilinear interpolation.
    const double cda00 = value_at(deployment_index, mach_index);
    const double cda01 = value_at(deployment_index, mach_index + 1);
    const double cda10 = value_at(deployment_index + 1, mach_index);
    const double cda11 = value_at(deployment_index + 1, mach_index + 1);

    // First interpolate along the deployment axis at the lower and upper Mach values.
    const double cda_at_lower_mach = cda00 + deployment_fraction_local * (cda10 - cda00);
    const double cda_at_upper_mach = cda01 + deployment_fraction_local * (cda11 - cda01);
    
    // Then interpolate between those two results along the Mach axis.
    return cda_at_lower_mach + mach_fraction_local * (cda_at_upper_mach - cda_at_lower_mach);
}

} // namespace airbrake