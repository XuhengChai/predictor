cmake_minimum_required(VERSION 3.5)
project(zf_ui_bsdw)

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

set(Qt5Core_DIR "/opt/qt5.12.3_aarch64/lib/cmake/Qt5Core/")
set(Qt5Quick_DIR "/opt/qt5.12.3_aarch64/lib/cmake/Qt5Quick/")
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(zf_global REQUIRED)
find_package(can_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(zf_cam_model REQUIRED)
find_package(Qt5 COMPONENTS Core Quick REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS highgui imgcodecs imgproc videoio core)
find_package(zf_framework_transport REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
	${rclcpp_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Quick_INCLUDE_DIRS}
)
message(${Qt5Quick_INCLUDE_DIRS})
message(${Qt5Core_INCLUDE_DIRS})

set(PROJECT_SOURCES
    src/main.cpp
    src/zf_bsd_model.cpp
    src/zf_paint_item.cpp
    src/qml.qrc
    include/zf_ui_bsdw/zf_bsd_gui_view.h
    include/zf_ui_bsdw/zf_bsd_obj_view.h
    include/zf_ui_bsdw/zf_bsd_view_model.h
    include/zf_ui_bsdw/zf_bsd_model.h
    include/zf_ui_bsdw/zf_paint_item.h
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# qt5_add_resources(QT_RESOURCES src/qml.qrc)
# add_executable(zf_ui_bsdw_exe src/main.cpp ${QT_RESOURCES})

message(${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_executable(${PROJECT_NAME} src/ui_bsdw.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
  "zf_global"
  "can_msgs"
  "interface"
  "zf_cam_model"
  "OpenCV"
  "zf_framework_transport"
)

target_link_libraries(${PROJECT_NAME}
	Qt5::Core
	Qt5::Quick
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
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
