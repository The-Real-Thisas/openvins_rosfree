add_library(ov_core_lib SHARED
        src/dummy.cpp
        src/cpi/CpiV1.cpp
        src/cpi/CpiV2.cpp
        src/sim/BsplineSE3.cpp
        src/track/TrackBase.cpp
        src/track/TrackAruco.cpp
        src/track/TrackDescriptor.cpp
        src/track/TrackKLT.cpp
        src/track/TrackSIM.cpp
        src/types/Landmark.cpp
        src/feat/Feature.cpp
        src/feat/FeatureDatabase.cpp
        src/feat/FeatureInitializer.cpp
        src/utils/print.cpp
)
target_include_directories(ov_core_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ov_core_lib PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        Boost::headers
        Boost::filesystem
        Boost::date_time
)

add_executable(test_webcam src/test_webcam.cpp)
target_link_libraries(test_webcam ov_core_lib)

add_executable(test_profile src/test_profile.cpp)
target_link_libraries(test_profile ov_core_lib)
