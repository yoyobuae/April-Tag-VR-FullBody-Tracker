# CMakeList.txt : CMake project for AprilTagTrackers, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

if (UNIX)
    add_definitions (-D__LINUX__)
endif (UNIX)

include_directories( ${OPENVR_INCLUDE_DIR} )

# Add source to this project's executable.
file(GLOB source "*.c" "*.cpp" "*.h" "*.hpp")
add_executable (AprilTagRouter ${source})

# TODO: Add tests and install targets if needed.

target_link_libraries( AprilTagRouter ${OPENVR_LIB} )
target_compile_features( AprilTagRouter PRIVATE cxx_std_17)
