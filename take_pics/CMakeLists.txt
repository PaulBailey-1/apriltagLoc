cmake_minimum_required(VERSION 2.8)

project( TakePics )

set(sources 
    "src/main.cpp"
)

include_directories("include/")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( takePics ${sources} )

target_link_libraries( takePics ${OpenCV_LIBS} )