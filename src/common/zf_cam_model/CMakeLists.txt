cmake_minimum_required(VERSION 3.5)
project(zf_cam_model)

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
# set(CMAKE_BUILD_TYPE "Debug")
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(zf_global REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#----# add library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/polynomial.cpp
  src/zf_cam_model_base.cpp
  src/zf_cam_model_scara.cpp
  src/zf_cam_model_wide.cpp
  src/zf_cam_model_truck.cpp
  src/zf_coordinate_transform.cpp
  src/zf_vision_trans_base.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "zf_global"
  "OpenCV"
  "zf_framework_transport"
)

add_executable(common_cam_model src/common_cam_model.cpp)
target_include_directories(common_cam_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(common_cam_model ${PROJECT_NAME})

# install(TARGETS common_cam_model
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  common_cam_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

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
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
