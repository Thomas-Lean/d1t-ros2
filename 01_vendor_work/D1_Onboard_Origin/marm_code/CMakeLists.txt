project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(marm_controller_node src/marm_controller_node.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp src/msg/SetServoAngle_.cpp src/msg/SetServoDumping_.cpp)
add_executable(marm_control_node src/marm_control_node.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp src/msg/SetServoAngle_.cpp src/msg/SetServoDumping_.cpp)
add_executable(marm_communication_node src/marm_communication_node.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp src/msg/SetServoAngle_.cpp src/msg/SetServoDumping_.cpp)
add_executable(publisher src/publisher.cpp src/msg/ArmString_.cpp)
add_executable(publisher1 src/publisher1.cpp src/msg/ArmString_.cpp)
add_executable(pub1 src/pub1.cpp src/msg/ArmString_.cpp)
add_executable(pub2 src/pub2.cpp src/msg/ArmString_.cpp)
add_executable(subscriber src/subscriber.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
add_executable(test src/test.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
target_link_libraries(marm_controller_node fsuartservo cserialport)