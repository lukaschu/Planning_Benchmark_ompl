cmake_minimum_required(VERSION 3.8)
project(benchmark_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  /opt/ros/humble/include
  /opt/ros/humble/include/ompl-1.6
  ${YAML_CPP_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
)

add_executable(simulation src/simulation_main.cpp)

add_library(${PROJECT_NAME}_NODE src/planning.cpp src/simulation.cpp src/utils.cpp src/collision_checker.cpp)

ament_target_dependencies(${PROJECT_NAME}_NODE rclcpp ompl Eigen3 moveit_ros_planning_interface moveit_msgs shape_msgs control_msgs yaml-cpp)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  simulation
  ${PROJECT_NAME}_NODE
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_NODE
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME}_NODE ${OMPL_LIBRARIES} Eigen3::Eigen yaml-cpp)

target_link_libraries(simulation ${PROJECT_NAME}_NODE)

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