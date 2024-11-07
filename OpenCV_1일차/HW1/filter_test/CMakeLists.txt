cmake_minimum_required(VERSION 3.0)
project(filter_test)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4  # 이 경로가 필요할 수 있음
    include
)

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${OpenCV_LIBS})
