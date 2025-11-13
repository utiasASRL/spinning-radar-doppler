add_library(spinning_radar_doppler
    utils/math.cpp
    utils/radar.cpp
    utils/imu.cpp
    extractor/doppler_extractor.cpp
    extractor/extractor_config.cpp
)

target_include_directories(spinning_radar_doppler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(spinning_radar_doppler
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        yaml-cpp
)

target_compile_features(spinning_radar_doppler PUBLIC cxx_std_17)

add_library(srd::core ALIAS spinning_radar_doppler)
