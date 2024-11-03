cmake_minimum_required(VERSION 3.8)
project(chatter_cli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# talker 실행 파일 설정
add_executable(talker src/talker.cpp)
target_include_directories(talker PRIVATE include)
ament_target_dependencies(talker rclcpp std_msgs)

# listener 실행 파일 설정
add_executable(listener src/listener.cpp)
target_include_directories(listener PRIVATE include)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

