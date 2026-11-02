cmake_minimum_required(VERSION 3.8)
project(d1t_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(grasp01_moveit_offline src/grasp01_moveit_offline.cpp)
target_compile_features(grasp01_moveit_offline PRIVATE cxx_std_17)
ament_target_dependencies(grasp01_moveit_offline moveit_core)
target_link_libraries(grasp01_moveit_offline nlohmann_json::nlohmann_json)
install(TARGETS grasp01_moveit_offline DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS launch/set_desk_collision.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME set_desk_collision)

install(PROGRAMS launch/grasp01_desk_visual.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME grasp01_desk_visual)

install(PROGRAMS launch/s4b4_tcp.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME s4b4_tcp)

install(PROGRAMS launch/s4b3_run10.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME s4b3_run10)

install(PROGRAMS launch/s4b3_clearance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME s4b3_clearance)


ament_package()
