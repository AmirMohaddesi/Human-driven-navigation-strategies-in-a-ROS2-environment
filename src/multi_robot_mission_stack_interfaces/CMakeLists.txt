cmake_minimum_required(VERSION 3.5)
project(multi_robot_mission_stack_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToPose.srv"
  "srv/GetNavigationState.srv"
  "srv/CancelNavigation.srv"
  "srv/NavigateToNamedLocation.srv"
  "srv/ProduceSemanticBlockedPassageV35.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

