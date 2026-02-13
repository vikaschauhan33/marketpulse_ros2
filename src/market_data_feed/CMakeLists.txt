cmake_minimum_required(VERSION 3.8)
project(market_data_feed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(marketpulse_interfaces REQUIRED)

# Add library for node implementation
add_library(market_data_feed_lib src/market_data_feed_node.cpp)
target_include_directories(market_data_feed_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(market_data_feed_lib PUBLIC cxx_std_20)
ament_target_dependencies(market_data_feed_lib
  rclcpp
  marketpulse_interfaces)

# Add executable
add_executable(market_data_feed_node src/main.cpp)
target_link_libraries(market_data_feed_node market_data_feed_lib)
ament_target_dependencies(market_data_feed_node
  rclcpp
  marketpulse_interfaces)

# Install targets
install(TARGETS market_data_feed_node
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
  ament_add_gtest(test_tick_generator test/test_tick_generator.cpp)
  target_link_libraries(test_tick_generator market_data_feed_lib)
  ament_target_dependencies(test_tick_generator
    rclcpp
    marketpulse_interfaces)
endif()

ament_package()
