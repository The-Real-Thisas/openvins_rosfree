add_library(ov_msckf_lib SHARED
        src/dummy.cpp
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/core/VioManagerHelper.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/update/UpdaterZeroVelocity.cpp
)
target_include_directories(ov_msckf_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ov_msckf_lib PUBLIC ov_init_lib)

add_executable(run_simulation src/run_simulation.cpp)
target_link_libraries(run_simulation ov_msckf_lib)

add_executable(test_sim_meas src/test_sim_meas.cpp)
target_link_libraries(test_sim_meas ov_msckf_lib)

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
target_link_libraries(test_sim_repeat ov_msckf_lib)
