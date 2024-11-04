cmake_minimum_required(VERSION 3.5)
project(image_analysis)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 자동 생성 설정 (Qt 사용 시)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)

# 소스 및 헤더 파일
file(GLOB PRO_SOURCE_FILES RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB PRO_INCLUDE_FILES RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.hpp include/${PROJECT_NAME}/*.h)
file(GLOB PRO_UI_FILES RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS ui/*.ui)
file(GLOB PRO_RESOURCE_FILES RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS resource/*.qrc)

# Qt-specific handling for UI, MOC, and resource files
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_FILES})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_FILES})
qt5_add_resources(QT_RESOURCE_HPP ${PRO_RESOURCE_FILES})

# Include directories 설정
include_directories(
  ${CMAKE_SOURCE_DIR}/include  # 기본 include 디렉터리
  ${CMAKE_SOURCE_DIR}/include/image_analysis  # image_analysis 하위 폴더
  ${CMAKE_CURRENT_BINARY_DIR}  # 빌드 파일이 생성되는 현재 디렉터리
)

# 실행 파일 추가
add_executable(${PROJECT_NAME}
  ${PRO_SOURCE_FILES}
  ${PRO_INCLUDE_FILES}
  ${QT_UI_HPP}
  ${QT_MOC_HPP}
  ${QT_RESOURCE_HPP}
)

# 의존성 및 라이브러리 링크
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs OpenCV cv_bridge Boost)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network ${OpenCV_LIBRARIES})

# 설치 파일 설정
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)
install(DIRECTORY ui/ DESTINATION share/${PROJECT_NAME}/ui)

# 테스트 설정 (옵션)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
