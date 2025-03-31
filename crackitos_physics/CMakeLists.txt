# crackitos_physics/CMakeLists.txt

# Collect all source and header files for the library
file(GLOB_RECURSE PHYSICS_SRC physics/src/*.cc)
file(GLOB_RECURSE PHYSICS_HEADERS physics/include/*.h)

# Create a static library for physics engine
add_library(crackitos_physics STATIC
        ${PHYSICS_SRC} ${PHYSICS_HEADERS}
)

# Include directories for the library
target_include_directories(crackitos_physics PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/physics/include
)

# Enable Tracy if profiling is enabled
if(ENABLE_PROFILING)
    target_link_libraries(crackitos_physics PUBLIC Tracy::TracyClient)
    target_compile_definitions(crackitos_physics PUBLIC TRACY_ENABLE=1)
endif(ENABLE_PROFILING)

# Compiler Warnings
if(MSVC)
    target_compile_options(crackitos_physics PRIVATE /W3 /WX)
else()
    target_compile_options(crackitos_physics PRIVATE -Wall -Wextra -Werror)
endif()
