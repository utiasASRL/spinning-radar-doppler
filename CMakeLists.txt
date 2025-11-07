cmake_minimum_required(VERSION 3.14)
project(spinning_radar_doppler LANGUAGES CXX)

# ---- Core dependencies ----
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(lgmath REQUIRED)

# ---- Subdirectories ----
add_subdirectory(src)

# ---- Optional tests ----
option(SRD_BUILD_TESTS "Build SRD unit tests" ON)
if(SRD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()