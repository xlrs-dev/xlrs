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

# Pure-logic sources that need a translation unit (the rest are header-only).
add_executable(xlrs_native_tests
    test_xlrs_native/test_main.cpp
    ${XLRS_LIB_DIR}/xlrs/link/Link.cpp
    ${XLRS_LIB_DIR}/xlrs/link/RfScheduler.cpp
    ${XLRS_LIB_DIR}/xlrs/timing/HwTimer.cpp
    ${XLRS_LIB_DIR}/xlrs/util/FlashBootCounter.cpp
)

target_include_directories(xlrs_native_tests PRIVATE
    ${XLRS_LIB_DIR}
    ${XLRS_LIB_DIR}/xlrs
)

target_link_libraries(xlrs_native_tests PRIVATE unity)

if(NOT MSVC)
    target_compile_options(xlrs_native_tests PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_test(NAME xlrs_native_tests COMMAND xlrs_native_tests)
