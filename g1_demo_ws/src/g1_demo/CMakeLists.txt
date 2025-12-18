cmake_minimum_required(VERSION 3.5)
project(g1_demo)

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

include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)


find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(loco_client_demo src/loco_client_demo.cpp)
# add_executable(loco_client_node src/loco_client_node.cpp)
add_executable(move_demo src/ai_demo.cpp)

target_compile_features(loco_client_demo PRIVATE cxx_std_20)
# target_compile_features(loco_client_node PRIVATE cxx_std_20)
target_compile_features(move_demo PRIVATE cxx_std_20)

ament_target_dependencies(loco_client_demo ${DEPENDENCY_LIST})
# ament_target_dependencies(loco_client_node ${DEPENDENCY_LIST})
ament_target_dependencies(move_demo ${DEPENDENCY_LIST} Eigen3 tf2 tf2_msgs tf2_ros)


install(TARGETS 
        loco_client_demo
        # loco_client_node
        move_demo
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

