cmake_minimum_required(VERSION 3.22)

project(examples CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ------------------------------------------------------------------------------
# find_package(pinocchio REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)
set(BUILD_SHARED_LIBS OFF)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
include(FetchContent)
FetchContent_Declare(
  ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG        v0.19.4 # or a specific version like v0.9.2
)
FetchContent_MakeAvailable(ruckig)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)

FetchContent_MakeAvailable(json)

# Foxglove C++ SDK: prebuilt dist (ships a static C lib + C++ wrapper source,
# no Rust toolchain needed to consume it). Live-streams telemetry to Foxglove
# Studio -- see foxgloveThread() in main.cpp.
set(FOXGLOVE_SDK_VERSION "0.27.0")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(FOXGLOVE_SDK_ASSET "foxglove-v${FOXGLOVE_SDK_VERSION}-cpp-aarch64-unknown-linux-gnu.zip")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(FOXGLOVE_SDK_ASSET "foxglove-v${FOXGLOVE_SDK_VERSION}-cpp-x86_64-unknown-linux-gnu.zip")
else()
  message(FATAL_ERROR "No prebuilt Foxglove SDK dist configured for "
    "${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}. See "
    "https://github.com/foxglove/foxglove-sdk/releases for other platforms.")
endif()

FetchContent_Declare(foxglove_sdk
  URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v${FOXGLOVE_SDK_VERSION}/${FOXGLOVE_SDK_ASSET}")
FetchContent_MakeAvailable(foxglove_sdk)

find_package(foxglove-sdk CONFIG REQUIRED HINTS "${foxglove_sdk_SOURCE_DIR}")
foxglove_sdk_add_cpp_library(foxglove_cpp TYPE STATIC)

find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

add_executable(parolController 
                src/main.cpp
                src/serialib.cpp)

target_include_directories(parolController PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${pinocchio_INCLUDE_DIRS})
target_link_libraries(parolController ruckig::ruckig Threads::Threads readline
                      Eigen3::Eigen pinocchio::pinocchio foxglove_cpp)

target_compile_definitions(parolController PRIVATE
  PAROL_URDF_PATH="src/assets/parol6-PGripper.urdf")


add_executable(parolFk src/kin-test.cpp)
target_include_directories(parolFk PRIVATE ${pinocchio_INCLUDE_DIRS})
target_link_libraries(parolFk PRIVATE Eigen3::Eigen pinocchio::pinocchio nlohmann_json::nlohmann_json)


target_compile_definitions(parolFk PRIVATE
  PAROL_URDF_PATH="src/assets/parol6-PGripper.urdf")
