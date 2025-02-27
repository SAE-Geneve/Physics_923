
# Profiling Option
option(ENABLE_PROFILING "Enable Tracy Profiling" OFF)
if (ENABLE_PROFILING)
    list(APPEND VCPKG_MANIFEST_FEATURES "profiling")
endif (ENABLE_PROFILING)

# GTest Option
option(ENABLE_GTESTS "Enable GTests" OFF)
if (ENABLE_GTESTS)
    list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif (ENABLE_GTESTS)

# Samples Option
option(ENABLE_SAMPLES "Enable Samples" OFF)
if (ENABLE_SAMPLES)
    list(APPEND VCPKG_MANIFEST_FEATURES "samples")
endif (ENABLE_SAMPLES)

cmake_minimum_required(VERSION 3.18)
project(PhysicsEngine)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(ENABLE_PROFILING)
    find_package(Tracy CONFIG REQUIRED)
endif(ENABLE_PROFILING)

if(ENABLE_GTESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
endif(ENABLE_GTESTS)

if(ENABLE_SAMPLES)
    find_package(SDL2 CONFIG REQUIRED)
    find_package(imgui CONFIG REQUIRED)
    add_subdirectory(samples)
endif(ENABLE_SAMPLES)

# Include all subdirectories
add_subdirectory(crackitos_physics)
