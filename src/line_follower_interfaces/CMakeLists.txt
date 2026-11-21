cmake_minimum_required(VERSION 3.8)
project(line_follower_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveMode.msg"
  "msg/PathPoint.msg"
  "msg/LineSegment.msg"
  "msg/Junction.msg"
  "msg/LineObservation.msg"
  "msg/PerceptionStatus.msg"
  "msg/LocalizationEstimate.msg"
  "msg/ObservedGraph.msg"
  "msg/LocalPath.msg"
  "msg/PathCandidate.msg"
  "msg/PathCandidates.msg"
  "msg/PathStatus.msg"
  "msg/VelocityCandidate.msg"
  "msg/MissionState.msg"
  "msg/SafetyStatus.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/RecordingStatus.msg"
  "msg/GcsSnapshot.msg"
  "msg/GcsLinkStatus.msg"
  "msg/GcsCommandRequest.msg"
  "msg/GcsCommandEvent.msg"
  "srv/BagCommand.srv"
  "srv/MissionCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
