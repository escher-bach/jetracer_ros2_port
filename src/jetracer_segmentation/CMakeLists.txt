cmake_minimum_required(VERSION 3.8)
project(jetracer_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# CUDA runtime: resolved through the /usr/local/cuda symlink (phase-1 image);
# only needed for cudaMalloc/cudaMemcpy - no nvcc, plain host compiler.
find_package(CUDAToolkit REQUIRED)

# TensorRT ships no CMake config; the phase-1 entrypoint links the real libs
# and headers into the standard multiarch dirs, so plain lookups suffice.
find_library(NVINFER_LIBRARY nvinfer REQUIRED)
find_path(NVINFER_INCLUDE_DIR NvInfer.h REQUIRED)

add_executable(segnet_node src/segnet_node.cpp)
target_compile_features(segnet_node PUBLIC cxx_std_17)
target_include_directories(segnet_node PRIVATE ${NVINFER_INCLUDE_DIR})

ament_target_dependencies(
  segnet_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
)
target_link_libraries(segnet_node ${NVINFER_LIBRARY} CUDA::cudart ${OpenCV_LIBS})

install(TARGETS segnet_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/ensure_model.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
