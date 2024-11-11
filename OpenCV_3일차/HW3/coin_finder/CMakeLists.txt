# CMakeLists.txt
cmake_minimum_required(VERSION 3.0)
project(coin_finder)

# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

# include 디렉토리 추가
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4  # OpenCV 설치 경로 (필요한 경우)
    include               # include 폴더 경로
)

# 소스 파일과 헤더 파일을 포함하여 실행 파일 생성
add_executable(coin_finder src/main.cpp src/coin_detector.cpp)

# OpenCV 라이브러리와 연결
target_link_libraries(coin_finder ${OpenCV_LIBS})

