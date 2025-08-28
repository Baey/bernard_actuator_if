cmake_minimum_required(VERSION 3.8)
project(bernard_actuator_if)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_INSTALL_RPATH $ORIGIN)  # sets proper runtime paths for exec to link .so properly

set(CANDLE_BUILD_STATIC ON)
set(CANDLE_OMIT_EXAMPLES ON)
add_subdirectory(candle)

add_executable(actuator_state_publisher src/actuator_state_publisher.cpp)
ament_target_dependencies(actuator_state_publisher rclcpp std_msgs sensor_msgs)
target_link_libraries(actuator_state_publisher candle)
target_include_directories(actuator_state_publisher PUBLIC ./candle/include ${CMAKE_SOURCE_DIR}/include)

install(TARGETS
  actuator_state_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
