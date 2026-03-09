cmake_minimum_required(VERSION 3.16)
project(image_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  sensor_msgs
  cv_bridge
  sensor_base
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  image_controller_parameters
  src/image_controller_parameters.yaml
)

add_library(image_controller SHARED
  src/image_controller.cpp
)
target_compile_features(image_controller PUBLIC cxx_std_17)
target_include_directories(image_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/image_controller>
)
target_link_libraries(image_controller PUBLIC
  image_controller_parameters
)
ament_target_dependencies(image_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(image_controller PRIVATE "IMAGE_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface image_controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  
endif()

install(
  DIRECTORY include/
  DESTINATION include/image_controller
)
install(
  TARGETS
    image_controller
    image_controller_parameters
  EXPORT export_image_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_image_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
