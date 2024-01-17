
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

if (UNIX)
    add_definitions (-D__LINUX__)
endif (UNIX)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${APRILTAG_INCLUDE_DIR} )
include_directories( ${OPENVR_INCLUDE_DIR} )
include_directories( "../V4L2Wrapper" )
include_directories( "../JPEGWrapper" )

file(GLOB v4l2source "../V4L2Wrapper/*.c" "../V4L2Wrapper/*.cpp" "../V4L2Wrapper/*.h" "../V4L2Wrapper/*.hpp")
file(GLOB jpegsource "../JPEGWrapper/*.c" "../JPEGWrapper/*.cpp" "../JPEGWrapper/*.h" "../JPEGWrapper/*.hpp")

message("v4l2source: ${v4l2source}")
message("jpegsource: ${jpegsource}")

# Add source to this project's executable.
file(GLOB source "*.c" "*.cpp" "*.h" "*.hpp")
add_executable (AprilTagTrackers ${source} ${v4l2source} ${jpegsource})

# TODO: Add tests and install targets if needed.

target_include_directories( AprilTagTrackers PRIVATE libjpeg-turbo::jpeg )

target_link_libraries( AprilTagTrackers libjpeg-turbo::jpeg)
target_link_libraries( AprilTagTrackers ${OPENVR_LIB} )
target_link_libraries( AprilTagTrackers ${OpenCV_LIBS} )
target_link_libraries( AprilTagTrackers apriltags)
target_link_libraries( AprilTagTrackers wx::base wx::core wx::net)
