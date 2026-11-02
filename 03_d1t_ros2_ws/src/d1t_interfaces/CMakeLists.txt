cmake_minimum_required(VERSION 3.8)
project(d1t_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointFeedback.msg"
  "msg/ArmStatus.msg"
  "msg/CommandAck.msg"
  "srv/SetServo.srv"
  "srv/SetOverride.srv"
  "srv/SetStiffness.srv"
  "srv/Grip.srv"
  "action/MoveToJoints.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
