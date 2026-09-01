cmake_minimum_required(VERSION 3.22)

project(examples CXX)


# ------------------------------------------------------------------------------
# find_package(pinocchio REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)

include(FetchContent)
FetchContent_Declare(
  ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG        v0.19.4 # or a specific version like v0.9.2
)
FetchContent_MakeAvailable(ruckig)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)

FetchContent_MakeAvailable(json)

find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

add_executable(parolController 
                src/main.cpp
                src/serialib.cpp)

target_include_directories(parolController PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(parolController ruckig::ruckig Threads::Threads readline)


add_executable(parolFk src/pinocchio.cpp)
target_include_directories(parolFk PRIVATE ${pinocchio_INCLUDE_DIRS})
target_link_libraries(parolFk PRIVATE Eigen3::Eigen pinocchio::pinocchio nlohmann_json::nlohmann_json)


target_compile_definitions(parolFk PRIVATE
  PAROL_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/assets/parol6.urdf")