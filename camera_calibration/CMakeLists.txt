cmake_minimum_required(VERSION 2.8)

project( CameraCalibration )

set(sources 
    "src/main.cpp"
)

include_directories("include/")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( cameraCal ${sources} )

target_link_libraries( cameraCal ${OpenCV_LIBS} )