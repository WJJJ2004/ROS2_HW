cmake_minimum_required(VERSION 3.0)
project(theta_finder)

# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

# 헤더 파일 경로 추가
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    include
)

# 소스 파일 정의
set(SOURCES
    src/ThetaFinder.cpp
    src/main.cpp
)

# 실행 파일 생성
add_executable(theta_finder ${SOURCES})

# OpenCV 라이브러리 연결
target_link_libraries(theta_finder ${OpenCV_LIBS})

