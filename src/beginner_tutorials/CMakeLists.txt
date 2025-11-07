cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindDifference.srv"
  )

# export dependencies
ament_export_dependencies(rosidl_default_runtime)

# add executables for listener and talker
add_executable(listener src/subscriber_member_function.cpp)
add_executable(talker src/publisher_member_function.cpp)

# target dependencies for listener and talker
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(talker rclcpp std_msgs)

# dependencies for listener and talker
add_dependencies(listener ${PROJECT_NAME})
add_dependencies(talker ${PROJECT_NAME})

# install talker, listener, and launch
install(TARGETS talker listener DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# link libraries
target_link_libraries(talker "${cpp_typesupport_target}")
target_link_libraries(listener "${cpp_typesupport_target}")
target_link_libraries( "${cpp_typesupport_target}")

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
