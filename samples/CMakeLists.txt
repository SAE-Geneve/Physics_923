# samples/CMakeLists.txt

if(ENABLE_SAMPLES)

	# Collect source and header files
	file(GLOB_RECURSE SAMPLES_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
	file(GLOB_RECURSE SAMPLES_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

	# Create a static library for SAMPLES
	add_library(samples STATIC ${SAMPLES_SRC} ${SAMPLES_HEADERS})

	# Include directories (Add `crackitos_physics` includes!)
	target_include_directories(samples PUBLIC
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_SOURCE_DIR}/crackitos_physics/common/include
			${CMAKE_SOURCE_DIR}/crackitos_physics/math/include
			${CMAKE_SOURCE_DIR}/crackitos_physics/physics/include
	)

	# Link dependencies
	target_link_libraries(samples PUBLIC SDL2::SDL2 SDL2::SDL2main imgui::imgui crackitos_physics)

	# Enable warnings
	if(MSVC)
		target_compile_options(samples PRIVATE /W3 /WX)
	else()
		target_compile_options(samples PRIVATE -Wall -Wextra -Werror)
	endif()

	# === SAMPLE EXECUTABLE ===
	add_executable(crackitos_samples ${CMAKE_CURRENT_SOURCE_DIR}/samples_main.cc)

	# Link crackitos_samples against samples and crackitos_physics
	target_link_libraries(crackitos_samples PRIVATE samples crackitos_physics)

endif(ENABLE_SAMPLES)
