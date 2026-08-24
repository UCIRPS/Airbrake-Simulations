#pragma once

#include <string>

#include "airbrake/drag_table.hpp"

namespace airbrake {

DragTable load_drag_table_csv(
    const std::string& file_path
);

}