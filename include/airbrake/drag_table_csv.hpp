#pragma once

#include <string>

#include "airbrake/drag_table.hpp"

namespace airbrake {

/**
    Loads a drag table from a CSV file.

    The CSV must contain deployment fraction, Mach number, and CdA columns.
    The loader validates the data and constructs a rectangular DragTable.

    @param file_path Path to the drag-table CSV file.
    @return Validated drag table ready for interpolation.

    @throws std::runtime_error if the file cannot be opened or parsed.
    @throws std::invalid_argument if the data is invalid.
 */
DragTable load_drag_table_csv(
    const std::string& file_path
);

} // namespace airbrake