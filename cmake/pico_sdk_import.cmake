# Import the Raspberry Pi Pico SDK.
#
# Preferred:
#   export PICO_SDK_PATH=$HOME/pico/pico-sdk
#
# Or let CMake fetch it into the build directory:
#   cmake -S . -B build -DPICO_SDK_FETCH_FROM_GIT=ON

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if (NOT PICO_SDK_PATH)
    if (PICO_SDK_FETCH_FROM_GIT)
        include(FetchContent)
        set(PICO_SDK_FETCH_TAG "master" CACHE STRING "Pico SDK git ref to fetch")
        FetchContent_Declare(
            pico_sdk
            GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
            GIT_TAG ${PICO_SDK_FETCH_TAG}
            GIT_SUBMODULES_RECURSE TRUE
        )
        FetchContent_MakeAvailable(pico_sdk)
        set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
    else()
        message(FATAL_ERROR "PICO_SDK_PATH is not set. Point it at a Raspberry Pi Pico SDK checkout or configure with -DPICO_SDK_FETCH_FROM_GIT=ON.")
    endif()
endif()

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
set(PICO_SDK_INIT_CMAKE_FILE "${PICO_SDK_PATH}/pico_sdk_init.cmake")

if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' does not appear to contain the Pico SDK")
endif()

include(${PICO_SDK_INIT_CMAKE_FILE})
