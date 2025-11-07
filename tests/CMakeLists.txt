# ---- GTest ----
find_package(GTest REQUIRED)

# ---- Executable ----
add_executable(test_math test_math.cpp)
add_executable(test_radar test_radar.cpp)
add_executable(test_imu test_imu.cpp)

# ---- Link to main library + GTest ----
target_link_libraries(test_math
    PRIVATE
        spinning_radar_doppler
        GTest::GTest
        GTest::Main
)
target_link_libraries(test_radar
    PRIVATE
        spinning_radar_doppler
        GTest::GTest
        GTest::Main
)
target_link_libraries(test_imu
    PRIVATE
        spinning_radar_doppler
        GTest::GTest
        GTest::Main
)

# ---- Register with CTest ----
add_test(NAME MathTests COMMAND test_math)
add_test(NAME RadarUtilsTests COMMAND test_radar)
add_test(NAME IMUTests COMMAND test_imu)