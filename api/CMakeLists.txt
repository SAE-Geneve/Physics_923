# api/CMakeLists.txt

if(ENABLE_SAMPLES)
	message(STATUS "Building API and main.cc sample application")

	# Collect source and header files
	file(GLOB_RECURSE API_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
	file(GLOB_RECURSE API_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

	# Create a static library for API
	add_library(api STATIC ${API_SRC} ${API_HEADERS})

	# Include directories (Add `lib` includes!)
	target_include_directories(api PUBLIC
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_SOURCE_DIR}/lib/common/include
			${CMAKE_SOURCE_DIR}/lib/math/include
			${CMAKE_SOURCE_DIR}/lib/physics/include
	)

	# Link dependencies
	target_link_libraries(api PUBLIC SDL2::SDL2 SDL2::SDL2main imgui::imgui lib)

	# Enable warnings
	if(MSVC)
		target_compile_options(api PRIVATE /W3 /WX)
	else()
		target_compile_options(api PRIVATE -Wall -Wextra -Werror)
	endif()

	# === SAMPLE EXECUTABLE ===
	add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

	# Link main against API and LIB
	target_link_libraries(main PRIVATE api lib)

endif(ENABLE_SAMPLES)
