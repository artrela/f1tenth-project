cmake_minimum_required(VERSION 3.5)
project(cpp_vo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  std_msgs
  visualization_msgs
  Eigen3
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_filters
  cv_bridge
  OpenCV
  )

  find_package(${PACKAGE} REQUIRED)

endforeach()

find_package(OpenCV 3.4.16 REQUIRED)

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)


# Create Cpp executable
add_executable(vo_node src/vo_node.cpp src/vo.cpp)
target_link_libraries(vo_node ${OpenCV_LIBS})
# target_link_libraries(vo_node ${EIGEN3_LIBS})

ament_target_dependencies(vo_node 
  ament_cmake 
  rclcpp
  std_msgs
  visualization_msgs
  Eigen3
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_filters
  cv_bridge
  OpenCV 
)

# Install Cpp executables
install(TARGETS
  vo_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
