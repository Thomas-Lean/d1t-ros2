cmake_minimum_required(VERSION 3.8)
project(d1t_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_arm REQUIRED)
find_package(d1t_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# --- Feedback node: subscribes /arm_Feedback, publishes 3 derived topics ---
add_executable(d1t_feedback_node src/d1t_feedback_node.cpp)
ament_target_dependencies(d1t_feedback_node rclcpp unitree_arm d1t_interfaces)
target_link_libraries(d1t_feedback_node nlohmann_json::nlohmann_json)

install(TARGETS d1t_feedback_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
