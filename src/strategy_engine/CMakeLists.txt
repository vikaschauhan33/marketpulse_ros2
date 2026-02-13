cmake_minimum_required(VERSION 3.8)
project(strategy_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(marketpulse_interfaces REQUIRED)

# Add library
add_library(strategy_engine_lib src/strategy_engine_node.cpp)
target_include_directories(strategy_engine_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(strategy_engine_lib PUBLIC cxx_std_20)
ament_target_dependencies(strategy_engine_lib
  rclcpp
  marketpulse_interfaces)

# Add executable
add_executable(strategy_engine_node src/main.cpp)
target_link_libraries(strategy_engine_node strategy_engine_lib)
ament_target_dependencies(strategy_engine_node
  rclcpp
  marketpulse_interfaces)

# Install targets
install(TARGETS strategy_engine_node
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
  ament_add_gtest(test_rolling_window test/test_rolling_window.cpp)
  target_include_directories(test_rolling_window PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_add_gtest(test_signal_generation test/test_signal_generation.cpp)
  target_include_directories(test_signal_generation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif()

ament_package()
