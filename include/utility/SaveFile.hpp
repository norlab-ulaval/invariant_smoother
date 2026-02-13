
#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include "estimator/RobotState_Smoother.hpp"

class SaveFile {
public:
    static void writeCSV(const std::string& filename, const std::vector<ROBOT_STATES>& states) {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
            return;
        }

        // Write header (optional, but good for CSV)
        // file << "R00,R01,R02,R10,R11,R12,R20,R21,R22,Px,Py,Pz,Vx,Vy,Vz" << std::endl;

        for (const auto& state : states) {
            // Rotation (9x1 flat)
            // Eigen matrices are column-major by default. 
            // The user requested "rotation matrix (9x1)". 
            // Usually this means flattening. 
            // If they want specific order (row-major vs col-major), standard for 9x1 is usually reading the matrix buffer directly or row-major.
            // Let's assume Row-Major for human readability in CSV unless specified.
            // However, Eigen stores in Column-Major. Map uses column-major by default.
            
            // Let's write element by element to be safe and clear.
            // Row-major: R(0,0), R(0,1), R(0,2), R(1,0)...
            
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    file << state.Rotation(r, c) << ",";
                }
            }

            // Position (3x1)
            file << state.Position(0) << ","
                 << state.Position(1) << ","
                 << state.Position(2) << ",";

            // Linear Velocity (3x1)
            file << state.Velocity(0) << ","
                 << state.Velocity(1) << ","
                 << state.Velocity(2);

            file << "\n";
        }

        file.close();
        std::cout << "Data successfully saved to " << filename << std::endl;
    }
};
