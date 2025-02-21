# lib/CMakeLists.txt

# Collect all source and header files for the library
file(GLOB_RECURSE COMMON_SRC common/src/*.cc)
file(GLOB_RECURSE MATH_SRC math/src/*.cc)
file(GLOB_RECURSE PHYSICS_SRC physics/src/*.cc)

file(GLOB_RECURSE COMMON_HEADERS common/include/*.h)
file(GLOB_RECURSE MATH_HEADERS math/include/*.h)
file(GLOB_RECURSE PHYSICS_HEADERS physics/include/*.h)

# Create a static library for physics engine
add_library(lib STATIC
        ${COMMON_SRC} ${COMMON_HEADERS}
        ${MATH_SRC} ${MATH_HEADERS}
        ${PHYSICS_SRC} ${PHYSICS_HEADERS}
)

# Include directories for the library
target_include_directories(lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/common/include
        ${CMAKE_CURRENT_SOURCE_DIR}/math/include
        ${CMAKE_CURRENT_SOURCE_DIR}/physics/include
)

# Link dependencies
target_link_libraries(lib PUBLIC SDL2::SDL2 imgui::imgui)

# Enable Tracy if profiling is enabled
if(ENABLE_PROFILING)
    target_link_libraries(lib PUBLIC Tracy::TracyClient)
    target_compile_definitions(lib PUBLIC TRACY_ENABLE=1)
endif()

# Compiler Warnings
if(MSVC)
    target_compile_options(lib PRIVATE /W3 /WX)
else()
    target_compile_options(lib PRIVATE -Wall -Wextra -Werror)
endif()

# === GOOGLE TEST SETUP ===

# Automatically find all test files inside lib/test/
file(GLOB_RECURSE TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc)

# Create test executable that compiles all tests together
add_executable(core_tests ${TEST_FILES})

# Link against lib and GTest
target_link_libraries(core_tests PRIVATE lib GTest::gtest GTest::gtest_main)

# Register tests with CTest
add_test(NAME CoreTests COMMAND core_tests)
