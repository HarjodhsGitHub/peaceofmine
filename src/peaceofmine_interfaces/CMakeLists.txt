cmake_minimum_required(VERSION 3.8)
project(peaceofmine_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/ActuatorStatus.msg"
  "msg/ServoCommand.msg"
  "msg/ServoTelemetry.msg"
  "msg/ServoState.msg"
  "msg/AdcSample.msg"
  "msg/AdcSamples.msg"
  "msg/ContactState.msg"
  "srv/ServoConfigure.srv"
  "srv/SaveSettings.srv"
  "action/ActuatorOperation.action"
  DEPENDENCIES std_msgs builtin_interfaces action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
