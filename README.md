# Invariant Smoother for Legged Robots

This repository contains an implementation of an Invariant Smoother for legged robot state estimation. It fuses data from IMU, kinematics (encoders), and contact sensors to estimate the robot's base state (position, velocity, orientation).

## Dependencies

*   **CMake**: Version 3.18 or higher.
*   **Eigen3**: Required for linear algebra operations.
*   **yaml-cpp** (optional): Used to parse, read, modify, and write YAML files. If you need YAML configuration or file handling, install `yaml-cpp` and update your `CMakeLists.txt` to call `find_package(yaml-cpp)` and link against it.
*   **C++ Compiler**: Must support C++17 standard.
*   **Linux OS**: Tested on Linux (as per environment).

## Build Instructions

1.  Clone the repository (if not already done).
2.  Navigate to the project root directory.
3.  Create a build directory and compile:

```bash
mkdir build
cd build
cmake ..
make -j4
```

## Usage

After a successful build, the executable `Invariant_Smoother_Test` will be generated.

To run the smoother:

```bash
./Invariant_Smoother_Test
```

### Configuration

The project uses CMake definitions to set the paths for data input and logging output. These are defined in `CMakeLists.txt`:

*   `DATA_PATH`: Directory containing input sensor data (default: `${CMAKE_SOURCE_DIR}/data`).
*   `LOG_PATH`: Directory where results will be saved (default: `${CMAKE_SOURCE_DIR}/log`).

Ensure these directories exist or update `CMakeLists.txt` to point to your desired locations.

## Running Example

The system is designed to read sensor data from a CSV file and output the estimated state.

### Input Data
The executable looks for a file named `sensor_data.csv` located in the `DATA_PATH`.
The CSV file format is expected to have **82 columns** per row, following this order:
1.  **IMU Angular Velocity** (3 columns: x, y, z)
2.  **IMU Acceleration** (3 columns: x, y, z)
3.  **Joint Positions** (12 columns)
4.  **Joint Velocities** (12 columns)
5.  **Contact State** (4 columns)
6.  **Foot Positions** (12 columns: 4 legs * 3 coords (x,y,z) in the body frame)
7.  **Foot Jacobians** (36 columns: 4 legs * 9 elements in the body frame)

:warning: Legs' order is 
* RR: rear right 
* RL: rear left
* FL: front left
* FR: front right

### Output Data
The results are saved to `result.csv` in the `LOG_PATH`.
The output CSV contains the estimated state evolution:
*   **Rotation Matrix** (9 columns)
*   **Position** (3 columns: x, y, z)
*   **Linear Velocity** (3 columns: x, y, z)

To plot the results, from the project root folder run:
```
python3 plot_results.py
```

## References

1.  Nisticò, Ylenia, Hajun Kim, João Carlos Virgolino Soares, Geoff Fink, Hae-Won Park, and Claudio Semini. "Multi-Sensor Fusion for Quadruped Robot State Estimation Using Invariant Filtering and Smoothing." IEEE Robotics and Automation Letters (2025).
    ```
    @ARTICLE{10977997,
      author={Nisticò, Ylenia and Kim, Hajun and Soares, João Carlos Virgolino and Fink, Geoff and Park, Hae-Won and Semini, Claudio},
      journal={IEEE Robotics and Automation Letters}, 
      title={Multi-Sensor Fusion for Quadruped Robot State Estimation Using Invariant Filtering and Smoothing}, 
      year={2025},
      volume={10},
      number={6},
      pages={6296-6303},
      keywords={Robots;Laser radar;Robot sensing systems;Legged locomotion;State estimation;Vectors;Odometry;Kinematics;Global Positioning System;Fuses;Sensor fusion;localization;legged robots},
      doi={10.1109/LRA.2025.3564711}}
    ```
2.  Yoon, Ziwon, Joon-Ha Kim, and Hae-Won Park. "Invariant smoother for legged robot state estimation with dynamic contact event information." *IEEE Transactions on Robotics* 40 (2023): 193-212.
    ```
    @ARTICLE{10301546,
    author={Yoon, Ziwon and Kim, Joon-Ha and Park, Hae-Won},
    journal={IEEE Transactions on Robotics}, 
    title={Invariant Smoother for Legged Robot State Estimation With Dynamic Contact Event Information}, 
    year={2024},
    volume={40},
    number={},
    pages={193-212},
    keywords={Robot sensing systems;Robots;Legged locomotion;Foot;State estimation;Odometry;Kinematics;Dynamic contact event;legged robots;localization;sensor fusion},
    doi={10.1109/TRO.2023.3328202}}
    ```

## License

This project is released under an open-source license. Use standard MIT or BSD-3 Clause terms if not otherwise specified.

## Contact

*   **Name**: Hajun Kim
*   **Email**: hajun0219@kaist.ac.kr
*   **Affiliation**: DRCD Lab, KAIST.
