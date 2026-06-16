cmake_minimum_required(VERSION 3.8)
project(odrive_pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
set(DEPENDENCIES
  ament_cmake
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  generate_parameter_library
  realtime_tools
  control_toolbox
  control_msgs
)

# Find dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate parameters
generate_parameter_library(odrive_pid_controller_parameters src/odrive_pid_controller_parameters.yaml)

# add controller library
add_library(${PROJECT_NAME} SHARED
  src/odrive_pid_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC odrive_pid_controller_parameters)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  ${DEPENDENCIES}
)
pluginlib_export_plugin_description_file(controller_interface odrive_pid_controller.xml)

# install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES odrive_pid_controller.xml DESTINATION share/${PROJECT_NAME})

ament_package()
