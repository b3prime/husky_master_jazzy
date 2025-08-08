cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# find dependencies
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Add source file executables
add_executable(occupancy_grid_node src/occupancy_grid_node.cpp)
target_compile_features(occupancy_grid_node PUBLIC cxx_std_17)

# Include and link PCL
target_include_directories(occupancy_grid_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(occupancy_grid_node ${PCL_LIBRARIES})

ament_target_dependencies(occupancy_grid_node
	rclcpp sensor_msgs nav_msgs pcl_conversions pcl_ros)

install(TARGETS occupancy_grid_node 
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME})

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
