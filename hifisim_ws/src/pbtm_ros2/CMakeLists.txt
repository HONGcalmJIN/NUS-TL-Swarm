cmake_minimum_required(VERSION 3.8)
project(pbtm_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(mavros_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rmw REQUIRED) 

# Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  libbspline/cpp/include
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Uncomment to set the build type and add compile options
# set(CMAKE_BUILD_TYPE "Release")
# add_compile_options(-std=c++11)
# add_compile_options(-std=c++14)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

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

# Add the executable and specify its dependencies
add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/pbtm.cpp
)

# Link against necessary libraries
# target_link_libraries(${PROJECT_NAME}_node 
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  rmw
  pcl_ros
  #Eigen3::Eigen
  rmw
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)