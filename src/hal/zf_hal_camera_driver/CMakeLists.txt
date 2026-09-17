cmake_minimum_required(VERSION 3.5)
project(zf_hal_camera_driver)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(sensor_msgs REQUIRED)
find_package(zf_framework_transport REQUIRED)
find_package(interface REQUIRED)

add_library(hal_camera_driver SHARED
  src/hal_camera_driver.cpp)
target_compile_definitions(hal_camera_driver
  PRIVATE "HAL_CAMERA_DRIVER_BUILDING_DLL")

target_include_directories(hal_camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# message(${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(
  hal_camera_driver
  "rclcpp"
  "rclcpp_components"
  "zf_global"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "zf_framework_transport"
  "interface"
)
# rclcpp_components_register_nodes(hal_camera_driver "zf::ImageConverter")
rclcpp_components_register_nodes(hal_camera_driver "zf::ImageConverter")
set(node_plugins "${node_plugins}zf::ImageConverter;$<TARGET_FILE:hal_camera_driver>\n")

# message("shoe file")
# message(node_plugins) --node_plugins
# message($<TARGET_FILE:hal_camera_driver>)

add_executable(hal_camera_driver_node src/hal_camera_driver_node.cpp)
target_link_libraries(hal_camera_driver_node
  hal_camera_driver
)
ament_target_dependencies(hal_camera_driver_node
  "rclcpp"
)

install(TARGETS
hal_camera_driver
hal_camera_driver_node
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS
#   hal_camera_driver
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(TARGETS hal_camera_driver_node
#   DESTINATION lib/${PROJECT_NAME})


# Install the header files
install(DIRECTORY include/
  DESTINATION include
)

  # EXPORT export_hal_camera_driver

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# # Install config files
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

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

# ament_export_include_directories(include)
# ament_export_targets(export_hal_camera_driver)

ament_export_libraries(hal_camera_driver)

ament_package()
