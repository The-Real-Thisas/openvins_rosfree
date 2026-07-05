add_library(ov_eval_lib SHARED
        src/dummy.cpp
        src/alignment/AlignTrajectory.cpp
        src/alignment/AlignUtils.cpp
        src/calc/ResultTrajectory.cpp
        src/calc/ResultSimulation.cpp
        src/utils/Loader.cpp
)
target_include_directories(ov_eval_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ov_eval_lib PUBLIC ov_core_lib)

foreach (tool
        format_converter
        error_comparison
        error_dataset
        error_singlerun
        error_simulation
        timing_comparison
        timing_flamegraph
        timing_histogram
        timing_percentages
        plot_trajectories)
    add_executable(${tool} src/${tool}.cpp)
    target_link_libraries(${tool} ov_eval_lib)
endforeach ()
