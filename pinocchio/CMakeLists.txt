cmake_minimum_required(VERSION 3.22)

project(examples CXX)
# --- ADD THIS SECTION (Find the Homebrew prefix and add it to the search path) ---
# Use 'cmake -DCMAKE_PREFIX_PATH=$(brew --prefix zeromq)' if you want to keep this clean.
# Or, hardcode the path (less flexible):
set(ZMQ_HOMEBREW_PREFIX "/opt/homebrew" CACHE PATH "Homebrew root for ZMQ") 
# Note: Use "/usr/local" if you are on an older Intel Mac!

set(CMAKE_PREFIX_PATH ${ZMQ_HOMEBREW_PREFIX} ${CMAKE_PREFIX_PATH})
# ------------------------------------------------------------------------------
find_package(pinocchio REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(controller src/main.cpp)
target_link_libraries(controller PRIVATE PkgConfig::ZeroMQ pinocchio::pinocchio)