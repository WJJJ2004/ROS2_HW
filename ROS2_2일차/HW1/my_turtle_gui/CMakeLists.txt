cmake_minimum_required(VERSION 3.5)
project(my_turtle_gui)

# 기본 설정
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)  # MOC 자동 생성 활성화
set(CMAKE_AUTORCC ON)  # 리소스 자동 생성 활성화

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(turtlesim REQUIRED)  # turtlesim 추가
find_package(std_srvs REQUIRED)   # std_srvs 추가

# 파일 경로 정의
set(PRO_SOURCE_FILES
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp
)
file(GLOB PRO_UI_FILES ui/mainwindow.ui)
file(GLOB PRO_INCLUDE_FILES include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
file(GLOB PRO_RESOURCE_FILES resource/images.qrc)

# Qt 처리
qt5_wrap_ui(QT_UI_HEADERS ${PRO_UI_FILES})
qt5_wrap_cpp(QT_MOC_HEADERS ${PRO_INCLUDE_FILES})
qt5_add_resources(QT_RESOURCES ${PRO_RESOURCE_FILES})

# include 디렉토리 설정
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
    ${PRO_SOURCE_FILES}
    ${QT_MOC_HEADERS}
    ${QT_RESOURCES}
    ${QT_UI_HEADERS}
)

# 타겟 의존성 추가
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  Boost
  turtlesim
  std_srvs
)

# Qt 라이브러리 링크
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::Network
)

# 설치 설정
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# 테스트 빌드 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 정보 설정
ament_package()

