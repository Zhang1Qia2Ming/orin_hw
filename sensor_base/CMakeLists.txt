cmake_minimum_required(VERSION 3.8)
project(sensor_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME}_lib SHARED
  src/sensor_base.cpp
)

target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${OpenCV_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME}_lib PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(${PROJECT_NAME}_lib PUBLIC rclcpp realtime_tools)
ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp realtime_tools)
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

ament_export_include_directories(include)
ament_package()
