cmake_minimum_required(VERSION 3.8)
project(vhit_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/vhit_robot_hardware_interface.cpp
  src/shared_memory_helper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_library(
  CTRLX_DATALAYER_LIB
  NAMES comm_datalayer
  REQUIRED
)

target_link_libraries(${PROJECT_NAME}
  ${CTRLX_DATALAYER_LIB}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    /usr/local/include/comm.datalayer
)

pluginlib_export_plugin_description_file(hardware_interface vhit_robot_hardware_interface_plugin.xml)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_vhit_robot_driver
    test/test_load_vhit_robot_driver.cpp
  )

  target_link_libraries(
    test_load_vhit_robot_driver
    ${PROJECT_NAME}
  )

  ament_target_dependencies(
    test_load_vhit_robot_driver
    hardware_interface
    pluginlib
    rclcpp
    ros2_control_test_assets
  )

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

# Install the shared library and header files
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES vhit_robot_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
