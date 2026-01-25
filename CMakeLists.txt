cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(oceanpad)

# 1. Add 'include' directory to the search path
target_include_directories(app PRIVATE include)

# 2. Automatically find all .cpp files in the src directory
file(GLOB app_sources src/*.cpp)

# 3. Add all found files to the build
target_sources(app PRIVATE ${app_sources})
