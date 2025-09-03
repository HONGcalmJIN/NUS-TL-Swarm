cmake_minimum_required(VERSION 3.8)
project(hifisim_msg_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Agent.msg"
  "msg/AgentFeasiblePose.msg"
  "msg/AgentGroup.msg"
  "msg/BoundingBox.msg"
  "msg/CollisionCrash.msg"
  "msg/Detection.msg"
  "msg/ImageCamInfo.msg"
  "msg/ImageCamInfoOoi.msg"
  "msg/IsCollision.msg"
  "msg/Measurement.msg"
  "msg/MeasurementOrg.msg"
  "msg/MissionResult.msg"
  "msg/ObjMeasured.msg"
  "msg/ObjsMeasurement.msg"
  "msg/OoiGroundTruth.msg"
  "msg/OoiGroundTruthFrame.msg"
  "msg/RadioBroadcast.msg"
  "msg/UavFOV.msg"
  "msg/Waypoint.msg"
  "srv/ActivateOoiService.srv"
  "srv/GetEnvBoundsService.srv"
  "srv/GetEnvCornersService.srv"
  "srv/GetEnvListService.srv"
  "srv/GetInitService.srv"
  "srv/GetMapService.srv"
  "srv/GetTotalAgentsService.srv"
  "srv/InitTestCaseService.srv"
  "srv/RadioBroadcastService.srv"
  "srv/ReachableAgentsService.srv"
  "srv/ResetEnvService.srv"
  "srv/ResetTestCaseService.srv"
  "srv/SimulationSetupService.srv"
  "srv/SpawnOoiService.srv"
  DEPENDENCIES 
  std_msgs 
  sensor_msgs
  geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
