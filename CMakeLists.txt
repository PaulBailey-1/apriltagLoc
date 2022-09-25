cmake_minimum_required(VERSION 2.8)

project( AprilTagLoc )

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(locator)
add_subdirectory(take_pics)
add_subdirectory(camera_calibration)