# main/CMakeLists.txt
add_executable(main main.cc)

# Include API headers
target_include_directories(main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link against API and LIB
target_link_libraries(main PRIVATE api lib)

# Enable testing
add_test(MainExecutable main)
