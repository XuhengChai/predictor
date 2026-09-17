cmake_minimum_required(VERSION 3.5)
project(zf_io)

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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#----# add library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/zf_io_session.cpp
  src/zf_io_ethernet.cpp
  )

add_executable(zf_io_exe src/zf_io.cpp)
target_include_directories(zf_io_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  zf_io_exe
  "rclcpp"
  "zf_global"
)
target_link_libraries(zf_io_exe
${PROJECT_NAME}
rt
)

# install(DIRECTORY include/
#   DESTINATION include
# )

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  ${PROJECT_NAME}
  zf_io_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
ament_export_libraries(${PROJECT_NAME}) # important if other pkg want to use it

ament_package()
