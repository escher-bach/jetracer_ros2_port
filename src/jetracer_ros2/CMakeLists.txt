cmake_minimum_required(VERSION 3.8)
project(jetracer_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(jetracer_node jetracer.cpp)
target_include_directories(jetracer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jetracer_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  jetracer_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
target_link_libraries(jetracer_node Boost::system Boost::thread)

install(TARGETS jetracer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
