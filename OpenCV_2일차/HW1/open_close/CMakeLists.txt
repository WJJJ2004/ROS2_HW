cmake_minimum_required(VERSION 3.0)
project(open_close)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4  # 이 경로가 필요할 수 있음
    include
)

add_executable(open_close src/open_close.cpp)
target_link_libraries(open_close ${OpenCV_LIBS})
