cmake_minimum_required(VERSION 3.16)

project(adc_cal_sim C)

option(ADC_CAL_ENABLE_SANITIZERS "Enable AddressSanitizer and UBSan" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(PROD_DIR "${CMAKE_CURRENT_LIST_DIR}/../test_platform/thesis_v3_500mhz_appl")

add_executable(adc_cal_sim
    sim_main.c
    sim_signal.c
    sim_dma.c
    sim_platform.c
    sim_tests.c
    "${PROD_DIR}/adc_frame.c"
    "${PROD_DIR}/adc_calibration_pipeline.c"
    "${PROD_DIR}/adc_calibration_dither.c"
    "${PROD_DIR}/adc_calibration_skew.c"
    "${PROD_DIR}/adc_calibration_performance.c"
    "${PROD_DIR}/calibration.c"
    "${PROD_DIR}/timing_alignment.c"
    "${PROD_DIR}/reference_buffer.c"
)

target_include_directories(adc_cal_sim PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${PROD_DIR}"
)

target_compile_definitions(adc_cal_sim PRIVATE
    ADC_CAL_SOFTWARE_SIMULATION=1
)

if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(adc_cal_sim PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow
    )
    if (ADC_CAL_ENABLE_SANITIZERS)
        target_compile_options(adc_cal_sim PRIVATE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
        )
        target_link_options(adc_cal_sim PRIVATE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
        )
    endif()
endif()

target_link_libraries(adc_cal_sim PRIVATE m)
