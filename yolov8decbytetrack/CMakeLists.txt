cmake_minimum_required(VERSION 3.5)
project(yolov8decbytetrack)

# ===================== C++ 标准 =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wno-deprecated-declarations)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ===================== CUDA / TensorRT =====================
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.9")
set(TensorRT_ROOT "/media/zqz/H/ubuntu_download/TensorRT-10.13.3.9")
set(msg_file "tutorial_inferfaces/msg/Detection.msg")
set(tutorial_interfaces_dir "/home/zqz/ros2_ws/tutorial_interfaces")

set(HIK_SDK_ROOT "/opt/MVS" CACHE PATH "Hikvision SDK root directory")
set(HIK_INCLUDE_DIR "${HIK_SDK_ROOT}/include")
set(HIK_LIB_DIR "${HIK_SDK_ROOT}/lib/64")
set(CONFIG_DIR "/home/zqz/ros2_ws/config")

find_package(CUDAToolkit REQUIRED)
find_package(CUDA 12.9 REQUIRED)

find_library(NVINFER_LIB
    NAMES nvinfer
    PATHS "${TensorRT_ROOT}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)
find_library(NVONNXPARSER_LIB
    NAMES nvonnxparser
    PATHS "${TensorRT_ROOT}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)
find_library(NVINFER_PLUGIN_LIB
    NAMES nvinfer_plugin
    PATHS "${TensorRT_ROOT}/lib"
    NO_DEFAULT_PATH
)

if(NOT EXISTS "${TensorRT_ROOT}/include/NvInfer.h")
    message(FATAL_ERROR "TensorRT headers not found in ${TensorRT_ROOT}/include")
endif()

# ===================== 验证海康威视SDK =====================
if(NOT EXISTS "${HIK_INCLUDE_DIR}")
    message(FATAL_ERROR "Hikvision SDK include directory not found: ${HIK_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${HIK_LIB_DIR}")
    message(FATAL_ERROR "Hikvision SDK lib directory not found: ${HIK_LIB_DIR}")
endif()

# ===================== ROS 2 =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tutorial_interfaces REQUIRED)  

# ===================== 第三方库 =====================
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)

# ===================== 自定义消息 =====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/Detection.msg"
  DEPENDENCIES geometry_msgs
)

# ===================== 源文件 =====================
set(SOURCES
    src/inference_node.cpp
    src/inference.cpp
    src/inference.h
    src/BYTETracker.cpp
    src/KalmanFilter.cpp
    src/lapjv.cpp
    src/Object.cpp
    src/Rect.cpp
    src/STrack.cpp
    src/HikCamera.cpp
)

# ===================== 可执行文件 =====================
add_executable(inference_node ${SOURCES})

# ===================== 包含目录 =====================
target_include_directories(inference_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${TensorRT_ROOT}/include
    ${CUDA_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${tutorial_interfaces_dir}
    ${HIK_INCLUDE_DIR}
    ${CONFIG_DIR}/include
)

# ===================== 链接库 =====================
target_link_libraries(inference_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
    ${NVINFER_PLUGIN_LIB}
    ${CUDA_LIBRARIES}
    ${rclcpp_LIBRARIES}
    cudart
    stdc++fs
    # === 海康威视库 - 根据实际文件名修改 ===
    ${HIK_LIB_DIR}/libMvCameraControl.so
    # 添加其他可能需要的库，根据实际文件名调整
    # ${HIK_LIB_DIR}/libMvDeviceManager.so
    # ${HIK_LIB_DIR}/libMvUsb3v.so
    # ${HIK_LIB_DIR}/libMvGigEVision.so
)

# ===================== ROS 2 依赖 =====================
ament_target_dependencies(inference_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tutorial_interfaces
)

# ===================== 安装 =====================
install(TARGETS inference_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(${rclcpp_INCLUDE_DIRS})
ament_export_libraries(${rclcpp_LIBRARIES})
ament_export_dependencies(rosidl_default_runtime)
ament_package()

# ===================== 调试信息 =====================
message(STATUS "TensorRT found at: ${TensorRT_ROOT}")
message(STATUS "nvinfer library: ${NVINFER_LIB}")
message(STATUS "nvonnxparser library: ${NVONNXPARSER_LIB}")
message(STATUS ">>> Found OpenCV version: ${OpenCV_VERSION}")
message(STATUS ">>> OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS ">>> OpenCV libs: ${OpenCV_LIBS}")
message(STATUS ">>> Hikvision SDK include: ${HIK_INCLUDE_DIR}")
message(STATUS ">>> Hikvision SDK lib: ${HIK_LIB_DIR}")