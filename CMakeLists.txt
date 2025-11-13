cmake_minimum_required(VERSION 3.14)
project(spinning_radar_doppler LANGUAGES CXX)

# ---- Global C++ Setup ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Dependencies ----
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- Library ----
add_subdirectory(src)

# ---- Optional Applications ----
option(SRD_BUILD_APPS "Build SRD command-line applications" OFF)
if(SRD_BUILD_APPS)
    add_subdirectory(apps)
endif()

# ---- Optional Tests ----
option(SRD_BUILD_TESTS "Build SRD test suite" OFF)
if(SRD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---- Install + Export ----
include(GNUInstallDirs)

# Install the library
install(
    TARGETS spinning_radar_doppler
    EXPORT spinning_radar_dopplerTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install public headers (the entire include/ folder)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export the CMake target
install(
    EXPORT spinning_radar_dopplerTargets
    FILE spinning_radar_dopplerTargets.cmake
    NAMESPACE srd::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spinning_radar_doppler
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/spinning_radar_dopplerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/spinning_radar_dopplerConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spinning_radar_doppler
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/spinning_radar_dopplerConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spinning_radar_doppler
)