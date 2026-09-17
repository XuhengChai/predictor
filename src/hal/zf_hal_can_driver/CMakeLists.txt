cmake_minimum_required(VERSION 3.5)
project(zf_hal_can_driver)

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

#----# find dependencies
set(THIS_PKG_DEPENDS
    rclcpp
    zf_global
    can_msgs
)
# foreach(Dependency IN ITEMS ${THIS_PKG_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# find_package(ament_cmake REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#----# add library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hal_can_id.cpp
  src/byte.cpp
  src/client/fake_can/fake_can_client.cpp
  src/client/pcan/pcan_client.cpp
  src/client/socket_can/socket_can_client.cpp
  src/client/ethernet_can/ethernet_can_client.cpp
  src/client/socket_can/can_filter.cpp
  src/client/can_client_factory.cpp
  src/client/can_client.cpp
  )
target_link_libraries(${PROJECT_NAME}
  pcanbasic
)

#----# add link library
ament_auto_add_library(hal_can_driver_node SHARED
  src/hal_can_driver_node.cpp
)

target_link_libraries(hal_can_driver_node
  ${PROJECT_NAME}
)
rclcpp_components_register_node(hal_can_driver_node
  PLUGIN "zf::driver::canbus::CanDriverNode"
  EXECUTABLE hal_can_driver_node_exe
)
#echo $LD_LIBRARY_PATH
set(node_plugins "${node_plugins}zf::driver::canbus::CanDriverNode;$<TARGET_FILE:hal_can_driver_node>\n")


# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# message(BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(hal_can_driver_exe src/hal_pcan_receive.cpp)
# add_executable(hal_can_driver_exe src/hal_can_driver_exe.cpp)
ament_target_dependencies(
  hal_can_driver_exe
  ${THIS_PKG_DEPENDS}
)
target_link_libraries(hal_can_driver_exe
  ${PROJECT_NAME}
)
target_include_directories(hal_can_driver_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${PROJECT_NAME}
  hal_can_driver_node
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
  ament_add_gtest(${PROJECT_NAME}_test
    src/test/hal_can_receiver_test.cpp
    # src/test/fake_can_client_test.cpp
    src/test/gtest_main.cpp
    )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

endif()
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(hal_can_driver_node)

ament_package()#setup.bash