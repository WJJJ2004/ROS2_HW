cmake_minimum_required(VERSION 3.5)
project(chatter_gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(std_msgs REQUIRED)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)  
set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)


set(SOURCES
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/main_window.cpp
    ${SOURCE_DIR}/talker_node.cpp
    ${SOURCE_DIR}/listener_node.cpp
)

set(UI_FILES
    ${UI_DIR}/mainwindow.ui
)

set(RESOURCES
    ${RESOURCE_DIR}/images.qrc
)

# Qt 파일 처리
qt5_wrap_ui(QT_UI_HPP ${UI_FILES})
qt5_wrap_cpp(QT_MOC_HPP
    ${INCLUDE_DIR}/chatter_gui/main_window.hpp
    ${INCLUDE_DIR}/chatter_gui/talker_node.hpp
    ${INCLUDE_DIR}/chatter_gui/listener_node.hpp
)
qt5_add_resources(QT_RESOURCE_HPP ${RESOURCES})

include_directories(
    ${INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${QT_UI_HPP}
    ${QT_MOC_HPP}
    ${QT_RESOURCE_HPP}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    Boost
    std_msgs
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Network
)


install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)  # 새로 추가된 부분

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
