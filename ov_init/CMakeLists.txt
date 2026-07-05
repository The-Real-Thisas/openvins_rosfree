add_library(ov_init_lib SHARED
        src/dummy.cpp
        src/ceres/Factor_GenericPrior.cpp
        src/ceres/Factor_ImageReprojCalib.cpp
        src/ceres/Factor_ImuCPIv1.cpp
        src/ceres/State_JPLQuatLocal.cpp
        src/init/InertialInitializer.cpp
        src/dynamic/DynamicInitializer.cpp
        src/static/StaticInitializer.cpp
        src/sim/SimulatorInit.cpp
)
target_include_directories(ov_init_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ov_init_lib PUBLIC
        ov_core_lib
        Ceres::ceres
)

add_executable(test_simulation src/test_simulation.cpp)
target_link_libraries(test_simulation ov_init_lib)

add_executable(test_dynamic_mle src/test_dynamic_mle.cpp)
target_link_libraries(test_dynamic_mle ov_init_lib)

add_executable(test_dynamic_init src/test_dynamic_init.cpp)
target_link_libraries(test_dynamic_init ov_init_lib)
