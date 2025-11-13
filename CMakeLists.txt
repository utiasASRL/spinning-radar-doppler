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
