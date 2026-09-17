cmake_minimum_required(VERSION 3.5)
project(zf_sys_manager)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(zf_global REQUIRED)
find_package(interface REQUIRED)

add_library(data_sys_manager SHARED
src/data_sys_manager.cpp)
target_compile_definitions(data_sys_manager
PRIVATE "DATA_SYS_MANAGER_DLL")
target_include_directories(data_sys_manager PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(data_sys_manager
"rclcpp"
"zf_global"
"rclcpp_components"
"interface")
rclcpp_components_register_node(data_sys_manager PLUGIN "zf::framework::SysManager" EXECUTABLE data_sys_manager_node)
install(TARGETS
data_sys_manager
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)

# add_library(data_sys_manager SHARED
#   src/data_sys_manager.cpp
# )
# target_compile_definitions(data_sys_manager
#   PRIVATE "DATA_SYS_MANAGER_DLL")

# target_include_directories(data_sys_manager PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(
#   data_sys_manager
#   "rclcpp"
#   "zf_global"
#   "interface"
# )

# # add_executable(data_sys_manager src/data_sys_manager.cpp)
# rclcpp_components_register_node(data_sys_manager PLUGIN "zf::framework::SysManager" EXECUTABLE data_sys_manager_node)
# set(node_plugins "${node_plugins}zf::framework::SysManager;$<TARGET_FILE:data_sys_manager>\n")

# # install(TARGETS data_sys_manager
# #   DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   data_sys_manager
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

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
