cmake_minimum_required(VERSION 3.8)
project(modot_raspi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(pigpio REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tactile_notifier.cpp
)
target_link_libraries(${PROJECT_NAME} pigpiod_if2)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "modot_raspi::TactileNotifier"
  EXECUTABLE tactile_notifier_node
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_auto_package()