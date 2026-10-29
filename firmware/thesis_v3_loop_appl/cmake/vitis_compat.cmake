# Application-local compatibility for a 2025.2 template on a 2025.1 BSP.
# Keep the vendor CMakeLists.txt and the exported platform unmodified.
include_guard(GLOBAL)

# Load the actual BSP helpers first. A newer BSP's native implementation wins.
find_package(common REQUIRED)

if(NOT COMMAND print_build_info_target)
    function(print_build_info_target target_name compiler_flags compile_defs linker_flags linker_command)
        add_custom_target(print_build_info ALL
            COMMAND "${CMAKE_COMMAND}" -E echo "Compiler flags: ${compiler_flags} ${compile_defs}"
            COMMAND "${CMAKE_COMMAND}" -E echo "Linker flags: ${linker_flags} ${linker_command}"
            COMMENT "Application build information (Vitis 2025.1 compatibility)"
            VERBATIM
        )
        add_dependencies(${target_name} print_build_info)
    endfunction()
    message(STATUS "Using application-local print_build_info_target for the Vitis 2025.1 BSP")
endif()
