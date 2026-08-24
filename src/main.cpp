#include "airbrake/drag_table_csv.hpp"

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char* argv[]){
    if (argc != 2){
        std::cerr << "Usage: airbrake_sim <drag-table.csv>\n";
        return 1;
    }

    try {
        const std::string file_path = argv[1];

        const airbrake::DragTable table = airbrake::load_drag_table_csv(file_path);

        std::cout << "Drag table loaded successfully\n";

        std::cout
            << "CdA at deployment 0.0, Mach 0.05: "
            << table.cda_m2(0.0, 0.05)
            << " m^2\n";

        std::cout
            << "CdA at deployment 1.0, Mach 0.70: "
            << table.cda_m2(1.0, 0.70)
            << " m^2\n";

        std::cout
            << "CdA at deployment 0.5, Mach 0.375: "
            << table.cda_m2(0.5, 0.375)
            << " m^2\n";


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}