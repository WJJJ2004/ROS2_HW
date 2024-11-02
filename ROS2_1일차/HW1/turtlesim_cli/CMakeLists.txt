cmake_minimum_required(VERSION 3.8)
project(turtlesim_cli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# include 디렉토리 설정 추가
include_directories(include)

# 실행 파일 및 소스 파일 정의
add_executable(turtlesim_cli_node src/turtlesim_cli_node.cpp)

# ament 패키지 의존성 설정
ament_target_dependencies(turtlesim_cli_node
  rclcpp
  turtlesim
  std_srvs
  geometry_msgs
)

# 실행 파일 설치 설정
install(TARGETS
  turtlesim_cli_node
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 의존성 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

