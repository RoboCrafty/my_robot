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

find_package(Threads REQUIRED)

add_executable(parolController 
                src/main.cpp
                src/serialib.cpp)

target_include_directories(parolController PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# target_link_libraries(controller PRIVATE PkgConfig::ZeroMQ pinocchio::pinocchio)
target_link_libraries(parolController ruckig::ruckig Threads::Threads)