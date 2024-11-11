cmake_minimum_required(VERSION 3.0)
project(contours)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4 
    include
)

add_executable(contours src/contours.cpp)
target_link_libraries(contours ${OpenCV_LIBS})
