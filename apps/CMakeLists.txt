add_executable(encode_chirps_in_radar_data encode_chirps_in_radar_data.cpp)

target_link_libraries(encode_chirps_in_radar_data
    PRIVATE
        spinning_radar_doppler
        ${OpenCV_LIBS}
        srd::core
        yaml-cpp
)