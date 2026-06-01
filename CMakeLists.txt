cmake_minimum_required(VERSION 3.20)
project(teleop_haptic_interface VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Build options ─────────────────────────────────────────────────────────────
# Build with Force Dimension Sigma 7 SDK.
# Set OFF to compile in simulation/mock mode (no haptic hardware required).
option(WITH_SIGMA "Build with Force Dimension Sigma 7 SDK support" ON)

# ─── vcpkg toolchain ───────────────────────────────────────────────────────────
# Override by passing -DCMAKE_TOOLCHAIN_FILE=<path> to cmake.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "vcpkg toolchain file")
endif()

# ─── External dependencies ────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Poco    REQUIRED COMPONENTS Foundation Net)
find_package(yaml-cpp REQUIRED)
find_package(msgpack-cxx CONFIG REQUIRED)

# ─── Source files ─────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp

    # Network layer (mirrored from avatar project — must stay in sync)
    src/network/udp_transport.cpp
    src/network/udp_reliable.cpp

    # Channels
    src/udp_arm_channel.cpp
    src/avatar_channel.cpp

    # Control
    src/passivity_controller.cpp
    src/teleop_controller.cpp
    src/teleop_session.cpp
    src/keyboard_haptic_device.cpp
)

# ─── Sigma 7 (Force Dimension) SDK ────────────────────────────────────────────
if(WITH_SIGMA)
    set(FD_SDK_DIR "C:/Program Files/Force Dimension/sdk-3.17.6"
        CACHE PATH "Force Dimension SDK root directory")

    find_library(FD_DHD_LIBRARY
        NAMES dhdms64 dhd
        PATHS "${FD_SDK_DIR}/lib"
        REQUIRED)
    find_library(FD_DRD_LIBRARY
        NAMES drdms64 drd
        PATHS "${FD_SDK_DIR}/lib"
        REQUIRED)
    find_path(FD_INCLUDE_DIR
        NAMES dhdc.h
        PATHS "${FD_SDK_DIR}/include"
        REQUIRED)

    list(APPEND SOURCES src/sigma_device.cpp)
    message(STATUS "[teleop] Building WITH Sigma 7 SDK: ${FD_SDK_DIR}")
else()
    message(STATUS "[teleop] Building WITHOUT Sigma 7 SDK (mock/sim mode)")
endif()

# ─── Executable ───────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include  # allows #include "network/common.hpp" etc.
    ${CMAKE_SOURCE_DIR}/src      # for any remaining local headers
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Eigen3::Eigen
    Poco::Foundation
    Poco::Net
    yaml-cpp::yaml-cpp
    msgpack-cxx              # INTERFACE IMPORTED — adds includes + MSGPACK_NO_BOOST define, no .lib
)

if(WITH_SIGMA)
    target_include_directories(${PROJECT_NAME} PRIVATE ${FD_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${FD_DHD_LIBRARY}
        ${FD_DRD_LIBRARY})
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_SIGMA)
endif()

# Winsock (needed by Poco Net on Windows even though Poco wraps it)
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32)
endif()

# ─── Compiler warnings ────────────────────────────────────────────────────────
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
