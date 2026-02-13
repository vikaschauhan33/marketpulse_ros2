cmake_minimum_required(VERSION 3.8)
project(risk_gate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(marketpulse_interfaces REQUIRED)

# Add library
add_library(risk_gate_lib src/risk_gate_node.cpp)
target_include_directories(risk_gate_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(risk_gate_lib PUBLIC cxx_std_20)
ament_target_dependencies(risk_gate_lib
  rclcpp
  marketpulse_interfaces)

# Add executable
add_executable(risk_gate_node src/main.cpp)
target_link_libraries(risk_gate_node risk_gate_lib)
ament_target_dependencies(risk_gate_node
  rclcpp
  marketpulse_interfaces)

# Install targets
install(TARGETS risk_gate_node
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_risk_limits test/test_risk_limits.cpp)
  target_link_libraries(test_risk_limits risk_gate_lib)
  ament_target_dependencies(test_risk_limits
    rclcpp
    marketpulse_interfaces)
endif()

ament_package()
