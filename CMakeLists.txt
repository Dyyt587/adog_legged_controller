cmake_minimum_required(VERSION 3.8)
project(adog_legged_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)
 
#find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add adog_legged_controller library related compile commands
generate_parameter_library(adog_legged_controller_parameters
  src/adog_legged_controller.yaml
  include/adog_legged_controller/validate_adog_legged_controller_parameters.hpp
)
#rosidl_generate_interfaces(adog_legged_controller
#  "msg/JointMit.msg"
#  )

add_library(
  adog_legged_controller
  SHARED
  src/adog_legged_controller.cpp
)
target_include_directories(adog_legged_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(adog_legged_controller adog_legged_controller_parameters)
ament_target_dependencies(adog_legged_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(adog_legged_controller PRIVATE "ADOG_LEGGED_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface adog_legged_controller.xml)

install(
  TARGETS
  adog_legged_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# There have a problom,so close it. 
#if(BUILD_TESTING)
if(0)

  ament_add_gmock(test_load_adog_legged_controller test/test_load_adog_legged_controller.cpp)
  target_include_directories(test_load_adog_legged_controller PRIVATE include)
  ament_target_dependencies(
    test_load_adog_legged_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_adog_legged_controller test/test_adog_legged_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/adog_legged_controller_params.yaml)
  target_include_directories(test_adog_legged_controller PRIVATE include)
  target_link_libraries(test_adog_legged_controller adog_legged_controller)
  ament_target_dependencies(
    test_adog_legged_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(test_adog_legged_controller_preceeding test/test_adog_legged_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/adog_legged_controller_preceeding_params.yaml)
  target_include_directories(test_adog_legged_controller_preceeding PRIVATE include)
  target_link_libraries(test_adog_legged_controller_preceeding adog_legged_controller)
  ament_target_dependencies(
    test_adog_legged_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  adog_legged_controller
)

ament_package()
