cmake_minimum_required(VERSION 2.8)

project( locator )

set(CMAKE_BUILD_TYPE Debug)

set(sources 
    "src/main.cpp"
    "src/Pose.cpp"
    "src/Display.cpp"
    "src/Detector.cpp"
    "src/Locator.cpp"
)

include_directories("include/")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( apriltag REQUIRED )
find_package( wpinet REQUIRED )
find_package( ntcore REQUIRED )
include_directories("/usr/local/include/ntcore")
include_directories("/usr/local/include/wpiutil")

add_executable( locator ${sources} )

target_compile_features( locator PUBLIC cxx_std_20)
target_link_libraries( locator apriltag ntcore ${OpenCV_LIBS})