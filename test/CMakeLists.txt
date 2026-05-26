# Native (host) test build for the XLRS pure-logic layers.
#
# This is a standalone CMake project — it deliberately does NOT pull in the
# Pico SDK. The compiled sources select their native fallbacks because neither
# XLRS_PICO_SDK nor PICO_BOARD is defined here, so the link/timing/flash logic
# runs against host stand-ins (simulated clock, in-RAM flash, MockPhy).
#
# Build and run:
#   cmake -S test -B build-tests
#   cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure
# or simply: scripts/test.sh

cmake_minimum_required(VERSION 3.14)
project(xlrs_tests C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0
)
FetchContent_MakeAvailable(unity)

set(XLRS_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../lib)

enable_testing()

set(XLRS_TEST_LIB_SOURCES
    ${XLRS_LIB_DIR}/xlrs/link/Link.cpp
    ${XLRS_LIB_DIR}/xlrs/link/RfScheduler.cpp
    ${XLRS_LIB_DIR}/xlrs/hal/FlashStore.cpp
    ${XLRS_LIB_DIR}/xlrs/timing/HwTimer.cpp
    ${XLRS_LIB_DIR}/xlrs/util/FlashBootCounter.cpp
)

function(add_xlrs_test suite)
    add_executable(xlrs_${suite}_tests
        ${suite}_tests.cpp
        ${XLRS_TEST_LIB_SOURCES}
        ${ARGN}
    )
    target_include_directories(xlrs_${suite}_tests PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${XLRS_LIB_DIR}
        ${XLRS_LIB_DIR}/xlrs
        ${XLRS_LIB_DIR}/crsfSerial
    )
    target_link_libraries(xlrs_${suite}_tests PRIVATE unity)
    if(NOT MSVC)
        target_compile_options(xlrs_${suite}_tests PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME xlrs_${suite}_tests COMMAND xlrs_${suite}_tests)
endfunction()

foreach(suite core timing scheduler link telemetry power crypto app flash)
    add_xlrs_test(${suite})
endforeach()

add_xlrs_test(crsf
    ${XLRS_LIB_DIR}/crsfSerial/crsfSerial.cpp
    ${XLRS_LIB_DIR}/crc8/crc8.cpp
)
