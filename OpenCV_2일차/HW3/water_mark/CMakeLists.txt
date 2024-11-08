cmake_minimum_required(VERSION 3.0)
project(water_mark)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4  # 이 경로가 필요할 수 있음
    include
)

add_executable(water_mark src/water_mark.cpp)
target_link_libraries(water_mark ${OpenCV_LIBS})
