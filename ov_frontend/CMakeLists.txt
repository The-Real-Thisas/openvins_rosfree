option(ENABLE_RERUN "Enable Rerun viewer visualization in run_dataset" ON)

add_executable(run_dataset
        src/run_dataset.cpp
        src/mcap_impl.cpp
)
target_include_directories(run_dataset PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mcap/include)
target_compile_definitions(run_dataset PRIVATE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
target_link_libraries(run_dataset ov_msckf_lib)

if (ENABLE_RERUN)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL
            https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
    target_link_libraries(run_dataset rerun_sdk)
    target_compile_definitions(run_dataset PRIVATE OV_HAVE_RERUN=1)
endif ()
