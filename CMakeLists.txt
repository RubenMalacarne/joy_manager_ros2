cmake_minimum_required(VERSION 3.8)
project(joy_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SwitchCommand.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(joystick_radiomaster_pocket_bl src/joystick_radiomaster_pocket_bl.cpp)
ament_target_dependencies(joystick_radiomaster_pocket_bl rclcpp sensor_msgs geometry_msgs std_msgs std_srvs)

add_executable(joystick_radiomaster_pocket src/joystick_radiomaster_pocket.cpp)
ament_target_dependencies(joystick_radiomaster_pocket rclcpp sensor_msgs geometry_msgs std_msgs std_srvs)

add_executable(mix_mode_button src/mix_mode_button.cpp)
ament_target_dependencies(mix_mode_button rclcpp sensor_msgs geometry_msgs std_msgs std_srvs)

# Make sure the generated interfaces are available
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(joystick_radiomaster_pocket_bl "${cpp_typesupport_target}")
target_link_libraries(joystick_radiomaster_pocket "${cpp_typesupport_target}")
target_link_libraries(mix_mode_button "${cpp_typesupport_target}")

add_executable(dualshock_ps4 src/dualshock_ps4.cpp)
ament_target_dependencies(dualshock_ps4 rclcpp sensor_msgs std_msgs std_srvs) 

install(TARGETS 
  joystick_radiomaster_pocket_bl
  joystick_radiomaster_pocket
  mix_mode_button
  dualshock_ps4
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  test/test_client_radiopocket.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch test
  DESTINATION share/${PROJECT_NAME}/
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
