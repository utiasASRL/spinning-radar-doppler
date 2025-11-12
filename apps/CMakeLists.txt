add_executable(encode_chirps_in_radar_data encode_chirps_in_radar_data.cpp)
add_executable(generate_doppler_velocity_csv generate_doppler_velocity_csv.cpp)

target_link_libraries(encode_chirps_in_radar_data
    PRIVATE
        spinning_radar_doppler
        ${OpenCV_LIBS}
        srd::core
        yaml-cpp
)

target_link_libraries(generate_doppler_velocity_csv
    PRIVATE
        spinning_radar_doppler
        ${OpenCV_LIBS}
        srd::core
        yaml-cpp
)