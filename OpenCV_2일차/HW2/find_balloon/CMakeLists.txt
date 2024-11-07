cmake_minimum_required(VERSION 3.0)
project(find_balloon)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4  # 이 경로가 필요할 수 있음
    include
)

add_executable(find_balloon src/find_balloon.cpp)
target_link_libraries(find_balloon ${OpenCV_LIBS})
