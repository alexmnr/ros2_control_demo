cmake_minimum_required(VERSION 3.8)
project(odrive_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
set(DEPENDENCIES
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Build the Shared Library
add_library(${PROJECT_NAME} SHARED
  src/odrive_hardware_interface.cpp
  src/joint.cpp
  src/odrive_base/socket_can.cpp
  src/odrive_base/epoll_event_loop.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the include headers
install(DIRECTORY include/
  DESTINATION include
)

# Install the plugin description file
install(FILES odrive_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ament Exports
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

ament_package()
