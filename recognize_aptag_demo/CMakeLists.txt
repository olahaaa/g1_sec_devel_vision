cmake_minimum_required(VERSION 3.8)
project(recognize_aptag_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(img_sub_node src/img_sub_node.cpp)
add_executable(tfsub_node src/tfsub_node.cpp)

ament_target_dependencies(img_sub_node
rclcpp
sensor_msgs
cv_bridge
)
ament_target_dependencies(tfsub_node
rclcpp
tf2_msgs
geometry_msgs
)

target_include_directories(img_sub_node 
PUBLIC
${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(img_sub_node
${OpenCV_LIBS}
)


install(TARGETS
img_sub_node
tfsub_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
