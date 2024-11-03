cmake_minimum_required(VERSION 3.8)
project(turtlesim_draw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)  

# include 디렉토리 추가
include_directories(include)

# add executable and source files
add_executable(turtlesim_draw_node src/turtlesim_draw_node.cpp src/turtlesim_draw.cpp)

ament_target_dependencies(turtlesim_draw_node rclcpp geometry_msgs turtlesim)

install(TARGETS
  turtlesim_draw_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

