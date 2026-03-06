cmake_minimum_required(VERSION 3.8)
project(t265_camera_base)

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

set(REALSENSE2_LIBRARY "/usr/local/lib/librealsense2.so")
set(REALSENSE2_INCLUDE_DIRS "/usr/local/include")


add_library(
  t265_camera_base
  SHARED
  src/t265_camera_sensor.cpp
)
target_include_directories(
  t265_camera_base
  PUBLIC
  include
  ${REALSENSE2_INCLUDE_DIRS}
)

ament_target_dependencies( t265_camera_base
 ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(t265_camera_base ${REALSENSE2_LIBRARY})

install(
  TARGETS
  t265_camera_base
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
target_link_libraries(hd_test t265_camera_base ${REALSENSE2_LIBRARY})
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
  t265_camera_base
)
ament_export_dependencies(
   ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
