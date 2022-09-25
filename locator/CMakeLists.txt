cmake_minimum_required(VERSION 2.8)

project( locator )

set(sources 
    "src/main.cpp"
)

include_directories("include/")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( apriltag REQUIRED )

add_executable( locator ${sources} )

target_link_libraries( locator apriltag ${OpenCV_LIBS})