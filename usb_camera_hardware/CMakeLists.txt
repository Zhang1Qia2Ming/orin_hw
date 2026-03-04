cmake_minimum_required(VERSION 3.8)
project(usb_camera_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_base
  OpenCV
  cv_bridge
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(
  usb_camera_hardware
  SHARED
  src/usb_camera_hw_interface.cpp
  src/usb_camera_sensor.cpp
)
target_include_directories(
  usb_camera_hardware
  PUBLIC
  include
)

ament_target_dependencies( usb_camera_hardware
 ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(usb_camera_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface usb_camera_hardware.xml)

install(
  TARGETS
  usb_camera_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(hd_test src/test.cpp) 
target_link_libraries(hd_test usb_camera_hardware)
ament_target_dependencies( hd_test
 ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS hd_test
DESTINATION lib/${PROJECT_NAME})

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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # ament_add_gmock(test_telebot_hardware_interface test/test_telebot_hardware_interface.cpp)
  # target_include_directories(test_telebot_hardware_interface PRIVATE include)
  # ament_target_dependencies(
  #   test_telebot_hardware_interface
  #   hardware_interface
  #   pluginlib
  #   ros2_control_test_assets
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  usb_camera_hardware
)
ament_export_dependencies(
   ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
