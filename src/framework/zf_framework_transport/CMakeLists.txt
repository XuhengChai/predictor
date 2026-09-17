cmake_minimum_required(VERSION 3.5)
project(zf_framework_transport)

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
  # src/shm/shm_public.cpp
  src/shm/block.cc
  src/shm/condition_notifier.cc
  src/shm/notifier_factory.cc
  src/shm/posix_segment.cc
  src/shm/readable_info.cc
  src/shm/rwlock.cc
  src/shm/segment.cc
  src/shm/segment_factory.cc
  src/shm/shm_conf.cc
  src/shm/state.cc
  src/shm/xsi_segment.cc
  src/shm/shm_public.cpp
  src/dispatcher/dispatcher.cc
  src/dispatcher/shm_dispatcher.cc
  src/message/identity.cc
  src/message/message_info.cc
  src/transport.cc
  )
# Link libraries
target_link_libraries(${PROJECT_NAME}
  pthread
  rt
  uuid
)

add_executable(zf_framework_transport_exe src/zf_framework_transport.cpp)
target_include_directories(zf_framework_transport_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  zf_framework_transport_exe
  "zf_global"
)
target_link_libraries(zf_framework_transport_exe
${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  zf_framework_transport_exe
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
  find_package(OpenCV 4 REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
  # src/test/condition_notifier_test.cc
  # src/test/identity_test.cc
  # src/test/message_info_test.cc
  # src/test/dispatcher_test.cc
  # src/test/shm_dispatcher_test.cc
  src/test/transport_test.cc
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}  ${OpenCV_LIBRARIES} )
endif()

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_export_libraries(${PROJECT_NAME})  # important if other pkg want to use it

ament_package()
