cmake_minimum_required(VERSION 3.8)
project(tonguinho)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)  # Inclua o cg_interfaces

# Add executable and link dependencies
add_executable(move_command_client src/move_command_client.cpp)
ament_target_dependencies(move_command_client rclcpp std_msgs cg_interfaces)  # Inclua cg_interfaces aqui

# Install the executable
install(TARGETS move_command_client
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

