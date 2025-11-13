cmake_minimum_required(VERSION 3.14)
project(spinning_radar_doppler VERSION 0.1.0 LANGUAGES CXX)

# ============================================================
#   Detect ROS2 / colcon (ament) build
# ============================================================
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    message(STATUS "Building spinning_radar_doppler in ROS2/colcon mode")

    # --- ROS-compatible dependency discovery ---
    find_package(eigen3_cmake_module REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(yaml-cpp REQUIRED)
    find_package(OpenCV REQUIRED)

    # --- Build library ---
    add_subdirectory(src)

    # --- Tests (ament handles BUILD_TESTING automatically) ---
    if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        add_subdirectory(tests)
    endif()

    # --- Install headers ---
    install(
        DIRECTORY include/
        DESTINATION include
    )

    # --- Install + export library ---
    install(
        TARGETS spinning_radar_doppler
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include
    )

    # --- Export to ROS2 ---
    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
    ament_export_dependencies(Eigen3 yaml-cpp OpenCV)

    # Finalize ROS2 package
    ament_package()

    return()  # ⬅ IMPORTANT: skip the normal CMake flow
endif()


# ============================================================
#   Normal (non-ROS) CMake flow
# ============================================================

message(STATUS "Building spinning_radar_doppler in normal CMake mode")

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

# ---- Optional Apps ----
option(SRD_BUILD_APPS "Build SRD command-line applications" ON)
if(SRD_BUILD_APPS)
    add_subdirectory(apps)
endif()

# ---- Optional Tests ----
option(SRD_BUILD_TESTS "Build SRD test suite" OFF)
if(SRD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ============================================================
#   Install + Export for normal CMake usage
# ============================================================
include(GNUInstallDirs)

# Install library + export
install(
    TARGETS spinning_radar_doppler
    EXPORT spinning_radar_dopplerTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install public headers
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets
install(
    EXPORT spinning_radar_dopplerTargets
    FILE spinning_radar_dopplerTargets.cmake
    NAMESPACE srd::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spinning_radar_doppler
)

# ============================================================
#   Generate and install Config.cmake (for find_package)
# ============================================================
include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/spinning_radar_dopplerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/spinning_radar_dopplerConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spinning_radar_doppler
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/spinning_radar_dopplerConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spinning_radar_doppler
)
