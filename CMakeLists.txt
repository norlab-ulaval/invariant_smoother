cmake_minimum_required(VERSION 3.18)

project(Invariant_Smoother LANGUAGES CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

option(USE_SYSTEM_LIBRARIES "Use system-installed libraries instead of local one" ON)
option(USE_LOCAL_THIRD_PARTY "Use local third party instead of the main third party" ON)


# Add source files
file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/estimator/*.cpp" "${PROJECT_SOURCE_DIR}/src/utility/*.cpp" "${PROJECT_SOURCE_DIR}/src/factor/*.cpp")

# Create a library target
add_library(Invariant_Smoother STATIC ${SOURCES})

if(${CMAKE_BUILD_TYPE} MATCHES Release)
    target_compile_options(Invariant_Smoother PRIVATE -O3 -march=native)
endif()


# Specify the include directory
target_include_directories(Invariant_Smoother PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )


target_link_libraries(Invariant_Smoother PRIVATE
        Eigen3::Eigen
        )



# set(DATA_PATH "${CMAKE_SOURCE_DIR}/data")
# set(LOG_PATH "${CMAKE_SOURCE_DIR}/log")
# add_compile_definitions(DATA_PATH=\"${DATA_PATH}\")
# add_compile_definitions(LOG_PATH=\"${LOG_PATH}\")


file(GLOB Invariant_Smoother_Test_src src/main.cpp)
set(TARGET Invariant_Smoother_Test)
add_executable(${TARGET} ${Invariant_Smoother_Test_src})

target_link_libraries(${TARGET} PRIVATE
        Eigen3::Eigen
        Invariant_Smoother
        )
target_include_directories(${TARGET} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# Define paths
set(DATA_PATH "${CMAKE_SOURCE_DIR}/data")
set(LOG_PATH "${CMAKE_SOURCE_DIR}/log")

# Compile-time definitions for the executable
target_compile_definitions(${TARGET} PRIVATE
    DATA_PATH="${DATA_PATH}"
    LOG_PATH="${LOG_PATH}"
)

# Ensure log directory exists at build time
add_custom_command(TARGET ${TARGET} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LOG_PATH}"
)

# Install library target
install(TARGETS Invariant_Smoother
        EXPORT Invariant_SmootherTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )

# Install headers
install(DIRECTORY include/ DESTINATION include/Invariant_Smoother)

# Export the targets
install(EXPORT Invariant_SmootherTargets
        FILE Invariant_SmootherTargets.cmake
        NAMESPACE Invariant_Smoother::
        DESTINATION lib/cmake/Invariant_Smoother
        )

# Install the Config.cmake
install(FILES
        cmake/Invariant_SmootherConfig.cmake
        DESTINATION lib/cmake/Invariant_Smoother
        )
