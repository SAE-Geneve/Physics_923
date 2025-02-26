
# Profiling Option
option(ENABLE_PROFILING "Enable Tracy Profiling" OFF)
if (ENABLE_PROFILING)
    list(APPEND VCPKG_MANIFEST_FEATURES "profiling")
endif (ENABLE_PROFILING)

cmake_minimum_required(VERSION 3.18)
project(PhysicsEngine)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()


# Dependencies (Find them via vcpkg)
find_package(SDL2 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

if(ENABLE_PROFILING)
    find_package(Tracy CONFIG REQUIRED)
endif()

# Include all subdirectories
add_subdirectory(lib)
add_subdirectory(api)
add_subdirectory(main)
