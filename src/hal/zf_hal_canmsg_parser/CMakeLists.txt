cmake_minimum_required(VERSION 3.5)
project(zf_hal_canmsg_parser)

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

#----# add library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dbc_iterator.cpp
  src/dbc_message.cpp
  src/dbc_signal.cpp
  src/hal_can_info.cpp
  src/hal_canmsg_parser.cpp
  )

#----# add link library
ament_auto_add_library(hal_canmsg_pp_node SHARED
  src/hal_canmsg_pp_node.cpp
)

target_link_libraries(hal_canmsg_pp_node
  ${PROJECT_NAME}
)
rclcpp_components_register_node(hal_canmsg_pp_node
  PLUGIN "zf::driver::canbus::CanMsgParserPackNode"
  EXECUTABLE hal_canmsg_pp_node_exe
)
#echo $LD_LIBRARY_PATH
set(node_plugins "${node_plugins}zf::driver::canbus::CanMsgParserPackNode;$<TARGET_FILE:hal_canmsg_pp_node>\n")

#----# add exe link library #echo $LD_LIBRARY_PATH
add_executable(dbc_test_exe test/dbc_test.cpp)
target_link_libraries(dbc_test_exe
  ${PROJECT_NAME}
)

add_executable(dbc_parser_test test/dbc_parser_test.cpp)
target_link_libraries(dbc_parser_test
  ${PROJECT_NAME}
)

#----# install 
install(TARGETS
  ${PROJECT_NAME}
  hal_canmsg_pp_node
  dbc_test_exe
  dbc_parser_test
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
# ament_export_libraries(${PROJECT_NAME})
# ament_export_libraries(hal_can_driver_node)

ament_package()
