#include "airbrake/drag_table_csv.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace airbrake {


/**
    Represents one parsed row from the drag-table CSV.

    The source line number is preserved so parsing errors can identify
    where the invalid data came from.
 */
    struct CsvEntry {
    double deployment;
    double mach;
    double cda;
    std::size_t line;
};

/**
    Returns true when a line contains only whitespace
*/
bool is_blank(const std::string& line){
    return line.find_first_not_of(" \t\r\n") == std::string::npos;
}

/**
    Parses one CSV row into a validated drag-table entry.

    Each row must contain exactly three comma-separated values;
    deployment_fraction, mach, cda_m2
*/
CsvEntry parse_entry(
    const std::string& line,
    std::size_t line_number
) {
    std::stringstream row_stream(line);

    CsvEntry entry{};
    char first_comma = '\0';
    char second_comma = '\0';

    // Read the three numeric values and the two separators.
    if (!(
        row_stream
        >> entry.deployment
        >> first_comma
        >> entry.mach
        >> second_comma
        >> entry.cda
    )) {
        throw std::invalid_argument("Invalid drag-table row at line " + std::to_string(line_number));
    }

    // Ensure the separators were actually commas.
    if (first_comma != ',' || second_comma != ','){
        throw std::invalid_argument(
            "Expected a three comma separated values at line " + std::to_string(line_number)
        );
    }
    
    // Reject rows containing additional values
    std::string extra_value;
    if (row_stream >> extra_value) {
        throw std::invalid_argument(
            "Too many values at line " + std::to_string(line_number)
        );
    }
    
    // Deployment and Mach bust be finite. CdA must also be non-negative.
    if (
        !std::isfinite(entry.deployment) || 
        !std::isfinite(entry.mach) || 
        !std::isfinite(entry.cda) || 
        entry.cda < 0.0
    ) {
        throw std::invalid_argument(
            "Non-finite or negative value at line " + std::to_string(line_number)
        );
    }
    entry.line = line_number;
    return entry;

} // namespace

DragTable load_drag_table_csv(
    const std::string& file_path
){
    std::ifstream input(file_path);

    if (!input){
        throw std::runtime_error(
            "Could not open drag-table file: " + file_path
        );
    }

    // The first line identifies the expected column order.
    std::string header;

    if (!std::getline(input, header)){
        throw std::invalid_argument(
            "Drag-table file is empty"
        );
    }

    // Remove the carriage return from Windows style line endings. 
    if (!header.empty() && header.back() == '\r'){
        header.pop_back();
    }

    if (header != "deployment_fraction,mach,cda_m2"){
        throw std::invalid_argument("Unexpected drag-table header");
    }

    std::vector<CsvEntry> entries;
    std::string line;
    std::size_t line_number = 1;

    while (std::getline(input, line)){
        ++line_number;

        // Blank lines do not represent table entries
        if (is_blank(line)){
            continue;
        }

        entries.push_back(parse_entry(line, line_number));
    }

    if (entries.empty()){
        throw std::invalid_argument("Drag table file contains on data");
    }

    // Sort rows so deployment values are grouped first and Mach values are ordered
    // within each deployment row
    std::sort(
        entries.begin(),
        entries.end(),
        [](const CsvEntry& left, const CsvEntry& right){
            if (left.deployment != right.deployment){
                return left.deployment < right.deployment;
            }
            return left.mach < right.mach;
        }
    );
    
    // A deployment/Mach pair must appear only once.
    for (std::size_t i = 1; i < entries.size(); ++i){
        if (entries[i].deployment == entries[i - 1].deployment && entries[i].mach == entries[i - 1].mach){
            throw std::invalid_argument(
                "Duplicate deployment/Mack pair near lines" + std::to_string(entries[i].line));
        }
    }

    std::vector<double> deployment_grid;
    std::vector<double> mach_grid;

    // Collect every deployment and Mach value from the parsed row
    for (const CsvEntry& entry : entries){
        deployment_grid.push_back(entry.deployment);
        mach_grid.push_back(entry.mach);
    }
    
    // Sort and remove duplicate axis values from interpolation grids. 
    std::sort(
        deployment_grid.begin(),
        deployment_grid.end()
    );

    deployment_grid.erase(
        std::unique(
            deployment_grid.begin(),
            deployment_grid.end()
        ),
        deployment_grid.end()
    );

    std::sort(
        mach_grid.begin(),
        mach_grid.end()
    );

    mach_grid.erase(
        std::unique(
            mach_grid.begin(),
            mach_grid.end()
        ),
        mach_grid.end()
    );

    // A valid table must contain every combination of deployment and Mach
    const std::size_t expected_values = deployment_grid.size() * mach_grid.size();

    if (entries.size() != expected_values){
        throw std::invalid_argument(
           "Drag-table is incomplete: expected "
            + std::to_string(expected_values)
            + " values but found "
            + std::to_string(entries.size())
        );
    }

    std::vector<double> cda_values;
    cda_values.reserve(expected_values);

    std::size_t entry_index = 0;

    // Store CdA values in row-major order:
    // 
    // deployment row 0: all Mach Values
    // deployment row 1: all Mach values
    //
    for (double deployment: deployment_grid){
        for (double mach : mach_grid){
            const CsvEntry& entry = entries[entry_index];

            // This catches missing grid pairs even when the total number of rows
            // happen to match the expected rectangular size.
            if (
                entry.deployment != deployment || entry.mach != mach
            ) {
                throw std::invalid_argument(
                    "Drag table is missing a deployment/Mack pair"
                );
            }

            cda_values.push_back(entry.cda);
            ++entry_index;
        }
    }

    // Move the completed vectors into DragTable rather than copying them.
    return DragTable(
        std::move(deployment_grid),
        std::move(mach_grid),
        std::move(cda_values)
    );
}


} // namespace airbrake