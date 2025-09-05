cmake_minimum_required(VERSION 3.8)
project(bernard_hw_if)

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

set(MAKE_TESTS OFF)
add_subdirectory(CANdle-SDK)

# Actuator state publisher
add_executable(actuator_state_publisher
 src/actuator_state_publisher.cpp
 src/actuators.cpp
)
ament_target_dependencies(actuator_state_publisher rclcpp std_msgs sensor_msgs)
target_link_libraries(actuator_state_publisher candle)
target_include_directories(actuator_state_publisher PUBLIC ./CANdle-SDK/candlelib/include ${CMAKE_SOURCE_DIR}/include)

# Actuator full interface
# add_executable(actuator_if
#  src/actuator_if.cpp
#  src/actuators.cpp
# )
# ament_target_dependencies(actuator_if rclcpp std_msgs sensor_msgs)
# target_link_libraries(actuator_if candle)
# target_include_directories(actuator_if PUBLIC ./candle/include ${CMAKE_SOURCE_DIR}/include)

install(TARGETS
  actuator_state_publisher
  # actuator_if
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
