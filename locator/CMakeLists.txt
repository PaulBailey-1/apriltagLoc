cmake_minimum_required(VERSION 2.8)

project( locator )

set(CMAKE_BUILD_TYPE Debug)

set(sources 
    "src/main.cpp"
    "src/Pose.cpp"
    "src/Display.cpp"
    "src/Detector.cpp"
    "src/Locator.cpp"
    "src/Stream.cpp"
    "src/Plane.cpp"
)

include_directories("include/")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( apriltag REQUIRED )
# find_package( wpiutil REQUIRED )
# find_package( ntcore REQUIRED )
find_package(realsense2 REQUIRED)
include_directories("/usr/local/include/ntcore")
include_directories("/usr/local/include/wpiutil")

add_executable( locator ${sources} )

target_compile_features( locator PUBLIC cxx_std_20)
target_link_libraries( locator apriltag /usr/local/lib/libntcore.so ${OpenCV_LIBS} ${realsense2_LIBRARY} ${PCL_LIBRARIES})
