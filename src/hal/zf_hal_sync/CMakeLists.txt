cmake_minimum_required(VERSION 3.5)
project(zf_hal_sync)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#----# add link library
ament_auto_add_library(hal_radar_node SHARED
  src/hal_radar_node.cpp
)
rclcpp_components_register_node(hal_radar_node
  PLUGIN "zf::driver::synchronization::HalRadarPackNode"
  EXECUTABLE hal_radar_node_exe
)
#echo $LD_LIBRARY_PATH
set(node_plugins "${node_plugins}zf::driver::synchronization::HalRadarPackNode;$<TARGET_FILE:hal_radar_node>\n")

#----# add link library
ament_auto_add_library(hal_sync_data_node SHARED
  src/hal_sync_data_node.cpp
)
rclcpp_components_register_node(hal_sync_data_node
  PLUGIN "zf::driver::synchronization::HalDataSyncNode"
  EXECUTABLE hal_sync_data_node_exe
)
#echo $LD_LIBRARY_PATH
set(node_plugins "${node_plugins}zf::driver::synchronization::HalDataSyncNode;$<TARGET_FILE:hal_sync_data_node>\n")

ament_auto_add_library(hal_sync_tracking_can_node SHARED
  src/hal_sync_tracking_can_node.cpp
)
rclcpp_components_register_node(hal_sync_tracking_can_node
  PLUGIN "zf::driver::synchronization::SyncTrackingCanNode"
  EXECUTABLE hal_sync_tracking_can_node_exe
)
#echo $LD_LIBRARY_PATH
set(node_plugins "${node_plugins}zf::driver::synchronization::SyncTrackingCanNode;$<TARGET_FILE:hal_sync_tracking_can_node>\n")

add_executable(hal_sync_exe src/hal_sync_exe.cpp)
target_include_directories(hal_sync_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  hal_sync_exe
  "rclcpp"
  "zf_global"
  "can_msgs"
  "interface"
  "zf_framework_transport"
  "zf_hal_can_driver"
  "zf_hal_camera_driver"
  "zf_hal_canmsg_parser"
)
target_link_libraries(hal_sync_exe
  hal_sync_data_node
  hal_radar_node
  hal_sync_tracking_can_node
)


install(TARGETS
  hal_radar_node
  hal_sync_data_node
  hal_sync_tracking_can_node
  hal_sync_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
