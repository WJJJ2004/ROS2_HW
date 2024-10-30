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

# 실행 파일 추가
add_executable(turtlesim_cli_node src/turtlesim_cli_node.cpp)

# 타겟 의존성 연결
ament_target_dependencies(turtlesim_cli_node
  rclcpp
  turtlesim
  std_srvs
  geometry_msgs
)

# 설치 경로 지정
install(TARGETS
  turtlesim_cli_node
  DESTINATION lib/${PROJECT_NAME})

# Linting & Testing 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스 및 저작권 검사 비활성화
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 설정
ament_package()

