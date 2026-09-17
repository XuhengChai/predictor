cmake_minimum_required(VERSION 3.5)
project(zf_detect_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
message(STATUS "Found OpenCV at: ${OpenCV_DIR}")
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(zf_global REQUIRED)
# find_package(zf_framework_transport REQUIRED)
# find_package(interface REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio opencv_dnn cudawarping)
# include_directories("/usr/include/eigen3")

list(FIND OpenCV_LIBS "cudawarping" CUDA_ARITHM_FOUND)
if(CUDA_ARITHM_FOUND EQUAL -1)
    message(WARNING "no CUDA module!")
endif()

# ============= cuda ============
find_package(CUDA REQUIRED)
include_directories($ENV{CUDA_DIR}/include)
link_directories($ENV{CUDA_DIR}/lib64)
#-------------------------------------------------------------------------------
# TensorRT
#-------------------------------------------------------------------------------
MESSAGE("Find TensorRT at $ENV{TENSORRT_DIR}")
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS $ENV{TENSORRT_DIR}
  PATH_SUFFIXES include)
find_library(TENSORRT_LIBRARY_INFER nvinfer
  HINTS $ENV{TENSORRT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(TENSORRT_LIBRARY_ONNX nvonnxparser
  HINTS $ENV{TENSORRT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
set (TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER} ${TENSORRT_LIBRARY_ONNX})
MESSAGE(STATUS "Find TensorRT libs at ${TENSORRT_LIBRARY}")
find_package_handle_standard_args(
  TENSORRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBRARY)
if(NOT TENSORRT_FOUND)
  message(ERROR
    "Cannot find TensorRT library.")
endif()
# # ============= TENSORRT not work ============
# include_directories(/home/nvidia/qt/TensorRT-8.2.5.1/include)
link_directories($ENV{TENSORRT_DIR}/lib)
# get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
# message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

#----# add library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lapjv.cpp
  src/zf_det_ego_angle.cpp
  src/zf_byte_tracker.cpp
  # src/zf_det_track_node.cpp
  src/kalmanFilter.cpp
  src/zf_track.cpp
)

add_executable(detect_tracking src/zf_det_track_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "zf_global"
  "zf_framework_transport"
  "zf_cam_model"
  "interface"
  "OpenCV"
)

target_link_libraries(detect_tracking ${PROJECT_NAME} nvinfer nvinfer_plugin cudart)

add_executable(zf_det_exe test/zf_det_exe.cpp)
target_link_libraries(zf_det_exe
  ${PROJECT_NAME}
  nvinfer nvinfer_plugin cudart
)

install(TARGETS
  ${PROJECT_NAME} 
  detect_tracking
  zf_det_exe
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

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_export_libraries(${PROJECT_NAME})  # important if other pkg want to use it

ament_package()
