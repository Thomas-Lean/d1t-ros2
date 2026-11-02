cmake_minimum_required(VERSION 3.8)
project(d1t_description)

find_package(ament_cmake REQUIRED)
find_package(fcl REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(model01_proximity src/model01_proximity.cpp)
target_compile_features(model01_proximity PRIVATE cxx_std_17)
target_link_libraries(model01_proximity fcl nlohmann_json::nlohmann_json)
install(TARGETS model01_proximity DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/joint_state_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_bridge)

install(PROGRAMS scripts/model01_analysis.py scripts/model01_view.py scripts/model01_render.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/model01_geometry.py
  DESTINATION lib/${PROJECT_NAME})


ament_package()
