# ---- GTest ----
find_package(GTest REQUIRED)

# ---- Executable ----
add_executable(test_radar test_radar.cpp)

# ---- Link to main library + GTest ----
target_link_libraries(test_radar
    PRIVATE
        srd::core
        GTest::GTest
        GTest::Main
)

# ---- Register with CTest ----
add_test(NAME RadarUtilsTests COMMAND test_radar)
